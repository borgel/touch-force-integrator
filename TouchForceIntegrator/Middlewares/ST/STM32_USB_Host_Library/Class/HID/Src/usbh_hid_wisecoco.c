/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

#include "usbh_hid_wisecoco.h"
#include "usbh_hid_parser.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
 *
 * HID Report decoder tables for the Elan 0x04F3/0x0732 touchscreen
 * (VID/PID) — a composite "Touchscreen" device with one HID interface
 * that exposes six Report IDs:
 *
 *   ReportID  Direction  Purpose
 *   --------  ---------  -------------------------------------------------
 *      1      Input      Multi-touch digitizer (10 fingers, scan time, contact count)
 *     10      Feature    Contact Count Maximum (how many simultaneous touches)
 *     68      Feature    Vendor certification blob (256 bytes, Usage Page 0xFF00)
 *      2      Input      Vendor opaque 64-byte payload (Usage Page 0x01FF)
 *      3      Output     Vendor opaque 63-byte payload (Usage Page 0xFF00)
 *      4      Input      Vendor opaque 19-byte payload (Usage Page 0xFF01)
 *
 * Each report lives in its own buffer (see buffer declarations below),
 * because every HID transfer corresponds to exactly one Report ID.
 *
 * The HID_Report_ItemTypedef structs in this file describe HOW to read
 * each individual field out of a received buffer: byte offset (via data
 * pointer), bit shift within that byte, bit size, sign, and the logical
 * and physical ranges declared by the device.
 *
 * NOTE on Report 1 field counts:
 *   The descriptor declares Report Count = 2 for both the X and Y
 *   Input items (and never resets it between them). Per the HID spec
 *   this means each finger carries TWO 16-bit X values and TWO 16-bit
 *   Y values — exposed below as prop_x / prop_x2 and prop_y / prop_y2.
 *   The device was verified empirically to emit the full 116-byte
 *   Report 1, which splits over USB FS as one 64-byte packet plus one
 *   52-byte packet on endpoint 0x81.
 */

/* =========================================================================
 *   Receive / transmit buffers, one per Report ID
 *
 *   These are where the USB stack deposits incoming reports (or where
 *   outgoing reports are staged). If your project already defines them
 *   elsewhere, change these to `extern` declarations and remove the
 *   definitions.
 * ========================================================================= */

enum PacketRxSm {
  PRS_ERR = 0,
  PRS_IDLE,     // waiting for data (or getting the first 64 byte chunk)
  PRS_PT2,      // getting the 52 byte chunk
  PRS_READY,    // have a full frame, ready to process it
};
/* ReportID 1: 10 fingers * 11 bytes + 4-byte scan time + 1-byte contact count = 115 bytes. */
// buffer we use to dequeue items from the FIFO into for us to process
#define REPORT_1_LEN    (REPORT_ID_OFFSET + 115)
uint8_t touch_report_data          [REPORT_1_LEN];
enum PacketRxSm touchReportState;

// buffer that the driver puts raw USB HID data into before putting it into the FIFO to send to us
uint8_t touch_rx_report_buf        [REPORT_ID_OFFSET + 75U + 1];

/* ReportID 10: 1-byte Contact Count Maximum. */
uint8_t feature_contact_max_data   [REPORT_ID_OFFSET + 1U];

/* ReportID 68 (0x44): 256-byte vendor certification blob. */
uint8_t feature_cert_blob_data     [REPORT_ID_OFFSET + 256U];

/* ReportID 2: 64-byte vendor input payload. */
uint8_t vendor_in_report2_data     [REPORT_ID_OFFSET + 64U];

/* ReportID 3: 63-byte vendor output payload. */
uint8_t vendor_out_report3_data    [REPORT_ID_OFFSET + 63U];

/* ReportID 4: 19-byte vendor input payload. */
uint8_t vendor_in_report4_data     [REPORT_ID_OFFSET + 19U];



/* =========================================================================
 *   ReportID 1 — Multi-Touch Digitizer (Input)
 *
 *   Layout within touch_report_data (offsets from REPORT_ID_OFFSET):
 *
 *     +0   .. +10    Finger  0   (11 bytes)
 *     +11  .. +21    Finger  1
 *     +22  .. +32    Finger  2
 *     +33  .. +43    Finger  3
 *     +44  .. +54    Finger  4
 *     +55  .. +65    Finger  5
 *     +66  .. +76    Finger  6
 *     +77  .. +87    Finger  7
 *     +88  .. +98    Finger  8
 *     +99  .. +109   Finger  9
 *     +110 .. +113   Scan Time       (uint32 LE, 100 microsecond ticks)
 *     +114           Contact Count   (number of fingers valid this frame)
 *
 *   Each 11-byte finger slot:
 *
 *     byte +0  bit 0       Tip Switch            (0 = no contact, 1 = contact)
 *     byte +0  bit 1       constant pad          (ignore)
 *     byte +0  bits 2..7   Contact Identifier    (0..63, stable per touch)
 *     byte +1              Width                 (contact patch width, 0..255)
 *     byte +2              Height                (contact patch height, 0..255)
 *     byte +3, +4          X1                    (uint16 LE, 0..2160)
 *     byte +5, +6          X2                    (uint16 LE, 0..2880 — second X field per descriptor)
 *     byte +7, +8          Y1                    (uint16 LE, 0..2160)
 *     byte +9, +10         Y2                    (uint16 LE, 0..2880 — second Y field per descriptor)
 *
 *   The reason there are two X and two Y values per finger is that the
 *   descriptor sets Report Count = 2 before the X Input item and never
 *   resets it before the Y Input item. X1 inherits the pre-change
 *   logical/physical max (2160/1660); by the time Y is declared the
 *   logical max has been bumped to 2880 and the physical max to 2940,
 *   which apply to both Y fields equally. In practice, inspect live
 *   data to decide which pair is the authoritative coordinate — the
 *   second pair is commonly either a duplicate or a firmware artifact.
 *
 *   Units: X/Y are declared with unit exponent -1 and unit 0x11
 *   (SI length, cm), so one logical count = 0.1 mm.
 * ========================================================================= */

#define TOUCH_FINGER_COUNT      10U
#define TOUCH_FINGER_STRIDE     11U

/* Pointer to byte `b` of finger `n`'s slot within touch_report_data. */
#define FINGER_BYTE(n, b)  \
  (touch_report_data + REPORT_ID_OFFSET + (n) * TOUCH_FINGER_STRIDE + (b))


/* Tip Switch (Digitizer Usage 0x42): 1-bit contact indicator per finger. */
static const HID_Report_ItemTypedef prop_tip[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 0), .size = 1, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 1, .physical_min = 0, .physical_max = 1, .resolution = 1 },
};

/*
 * Contact Identifier (Digitizer Usage 0x51): 6 bits starting at bit 2
 * of byte 0 (bit 1 is constant padding the firmware inserts). A stable
 * ID that stays with the same physical finger across frames until lift.
 */
static const HID_Report_ItemTypedef prop_contact_id[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 0), .size = 6, .shift = 2, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 63, .physical_min = 0, .physical_max = 63, .resolution = 1 },
};

/* Width  (Digitizer Usage 0x48): 8-bit contact-patch width,  byte 1 of each finger. */
static const HID_Report_ItemTypedef prop_width[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 1), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
};

/* Height (Digitizer Usage 0x49): 8-bit contact-patch height, byte 2 of each finger. */
static const HID_Report_ItemTypedef prop_height[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 2), .size = 8, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 255, .physical_min = 0, .physical_max = 255, .resolution = 1 },
};

/*
 * X1 (Generic Desktop Usage 0x30, first of two): 16-bit little-endian, bytes 3-4.
 * Logical 0..2160, physical 0..1660 (device units), unit = 0.1 mm.
 * This is the field declared BEFORE the logical/physical max get bumped
 * up for Y, so it retains the "smaller" X range.
 */
static const HID_Report_ItemTypedef prop_x[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 3), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
};

/*
 * X2 (Generic Desktop Usage 0x30, second of two): 16-bit little-endian, bytes 5-6.
 * The Report Count = 2 from the X Input item creates this duplicate field.
 * Its logical/physical ranges are inherited from the time of declaration,
 * same as X1. Inspect live data to determine if this carries independent
 * meaning (e.g. a secondary sensor) or is just a duplicate of X1.
 */
static const HID_Report_ItemTypedef prop_x2[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2160, .physical_min = 0, .physical_max = 1660, .resolution = 1 },
};

/*
 * Y1 (Generic Desktop Usage 0x31, first of two): 16-bit little-endian, bytes 7-8.
 * Logical 0..2880, physical 0..2940 (device units), unit = 0.1 mm.
 * The logical and physical maxes were bumped after the X Input item, so
 * both Y1 and Y2 use the larger Y range.
 */
static const HID_Report_ItemTypedef prop_y[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 7), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
};

/*
 * Y2 (Generic Desktop Usage 0x31, second of two): 16-bit little-endian, bytes 9-10.
 * The Report Count = 2 from the Y Input item creates this duplicate field.
 */
static const HID_Report_ItemTypedef prop_y2[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 9), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
};

/*
 * Scan Time (Digitizer Usage 0x56): 32-bit counter in 100 us units since
 * an arbitrary start, wrapping at 0x7FFFFFFF. Lives at byte offset 110 of
 * the finger block.
 */
static const HID_Report_ItemTypedef prop_scan_time =
{
  .data         = touch_report_data + REPORT_ID_OFFSET + 110U,
  .size         = 32,
  .shift        = 0,
  .count        = 0,
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 0x7FFFFFFFU,
  .physical_min = 0,
  .physical_max = 0x7FFFFFFFU,
  .resolution   = 1
};

/*
 * Contact Count (Digitizer Usage 0x54): 8-bit number of fingers whose
 * slots contain valid data in this frame. Lives at byte 114.
 */
static const HID_Report_ItemTypedef prop_contact_count =
{
  .data         = touch_report_data + REPORT_ID_OFFSET + 114U,
  .size         = 8,
  .shift        = 0,
  .count        = 0,
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 127,
  .physical_min = 0,
  .physical_max = 127,
  .resolution   = 1
};


/* =========================================================================
 *   ReportID 10 — Feature — Contact Count Maximum
 *
 *   The host reads this once at enumeration to learn how many simultaneous
 *   touch points this device supports. Declared logical max is 10, matching
 *   the 10 finger slots in Report 1.
 * ========================================================================= */

static const HID_Report_ItemTypedef prop_contact_count_max =
{
  .data         = feature_contact_max_data + REPORT_ID_OFFSET,
  .size         = 8,
  .shift        = 0,
  .count        = 0,
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 10,
  .physical_min = 0,
  .physical_max = 10,
  .resolution   = 1
};


/* =========================================================================
 *   ReportID 68 (0x44) — Feature — Vendor Certification Blob
 *
 *   Usage Page 0xFF00, Usage 0xC5. 256 opaque bytes, typically used to
 *   carry Microsoft's Windows Precision Touchscreen certification blob
 *   (a magic string the HID class driver reads to unlock Precision
 *   Touch semantics). Treat as an array of 256 unsigned bytes.
 * ========================================================================= */

static const HID_Report_ItemTypedef prop_cert_blob =
{
  .data         = feature_cert_blob_data + REPORT_ID_OFFSET,
  .size         = 8,       /* bits per element */
  .shift        = 0,
  .count        = 255,     /* 256 elements, but .count is uint8_t so clamps to 255; */
                           /* read prop_cert_blob.data[i] directly if you need all 256 */
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 255,
  .physical_min = 0,
  .physical_max = 255,
  .resolution   = 1
};


/* =========================================================================
 *   ReportID 2 — Input — Vendor opaque (Usage Page 0x01FF, Usage 0x01)
 *
 *   64 bytes of vendor-defined input data. No published field meaning —
 *   just a raw buffer the application layer interprets.
 * ========================================================================= */

static const HID_Report_ItemTypedef prop_vendor_in2 =
{
  .data         = vendor_in_report2_data + REPORT_ID_OFFSET,
  .size         = 8,
  .shift        = 0,
  .count        = 64,
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 255,
  .physical_min = 0,
  .physical_max = 255,
  .resolution   = 1
};


/* =========================================================================
 *   ReportID 3 — Output — Vendor opaque (Usage Page 0xFF00, Usage 0x01)
 *
 *   63 bytes of vendor-defined output data, host-to-device. Typical use:
 *   commands or firmware config the host pushes to the touchscreen.
 * ========================================================================= */

static const HID_Report_ItemTypedef prop_vendor_out3 =
{
  .data         = vendor_out_report3_data + REPORT_ID_OFFSET,
  .size         = 8,
  .shift        = 0,
  .count        = 63,
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 255,
  .physical_min = 0,
  .physical_max = 255,
  .resolution   = 1
};


/* =========================================================================
 *   ReportID 4 — Input — Vendor opaque (Usage Page 0xFF01, Usage 0x01)
 *
 *   19 bytes of vendor-defined input data.
 * ========================================================================= */

static const HID_Report_ItemTypedef prop_vendor_in4 =
{
  .data         = vendor_in_report4_data + REPORT_ID_OFFSET,
  .size         = 8,
  .shift        = 0,
  .count        = 19,
  .sign         = 0,
  .logical_min  = 0,
  .logical_max  = 255,
  .physical_min = 0,
  .physical_max = 255,
  .resolution   = 1
};


/*
 * Latest published touch frame. Producer (USBH_HID_PumpTouchReports,
 * running in the USB host's task context) builds a fresh frame on its
 * own stack and then atomically copies it into latestData under a
 * brief critical section. Consumer (USBH_HID_WisecocoWaitFrame) wakes
 * on the semaphore and copies latestData out under the same critical
 * section. The critical sections are ~1 microsecond on Cortex-M7;
 * neither side ever holds the lock during USB parsing or rendering.
 */
static struct USBH_LatestWisecocoData latestData;
static StaticSemaphore_t s_newFrameSemMeta;
static SemaphoreHandle_t s_newFrameSem;

void USBH_HID_WisecocoAppInit(void)
{
  if (s_newFrameSem == NULL)
  {
    s_newFrameSem = xSemaphoreCreateBinaryStatic(&s_newFrameSemMeta);
  }
}

bool USBH_HID_WisecocoWaitFrame(struct USBH_LatestWisecocoData *out, uint32_t timeoutMs)
{
  if (s_newFrameSem == NULL || out == NULL)
  {
    return false;
  }
  TickType_t ticks = (timeoutMs == HAL_MAX_DELAY) ? portMAX_DELAY
                                                  : pdMS_TO_TICKS(timeoutMs);
  if (xSemaphoreTake(s_newFrameSem, ticks) != pdTRUE)
  {
    return false;
  }
  taskENTER_CRITICAL();
  *out = latestData;
  taskEXIT_CRITICAL();
  return true;
}

// User-configured rotation, applied per-finger inside USBH_HID_PumpTouchReports.
// Persists across re-enumeration; only the user changes it.
static USBH_WC_Rotation s_rotation = USBH_WC_ROTATE_0;

void USBH_HID_WisecocoSetRotation(USBH_WC_Rotation rotation)
{
  s_rotation = rotation;
}

/*
 * Apply s_rotation to one finger's raw (x, y) and patch dimensions in
 * place. Native coordinate system is x in [0, DISP_WIDTH_PX] and y in
 * [0, DISP_HEIGHT_PX]; rotated frame may swap those bounds.
 */
static void apply_rotation(struct USBH_WCSingleFinger *f)
{
  uint16_t x = f->x;
  uint16_t y = f->y;
  uint8_t pw = f->patchWidth;
  uint8_t ph = f->patchHeight;

  switch (s_rotation)
  {
    case USBH_WC_ROTATE_0:
    default:
      f->xFrac = (double)x / (double)DISP_WIDTH_PX;
      f->yFrac = (double)y / (double)DISP_HEIGHT_PX;
      break;

    case USBH_WC_ROTATE_90:
      f->x = (uint16_t)(DISP_HEIGHT_PX - y);
      f->y = x;
      f->patchWidth  = ph;
      f->patchHeight = pw;
      f->xFrac = 1.0 - (double)y / (double)DISP_HEIGHT_PX;
      f->yFrac = (double)x / (double)DISP_WIDTH_PX;
      break;

    case USBH_WC_ROTATE_180:
      f->x = (uint16_t)(DISP_WIDTH_PX  - x);
      f->y = (uint16_t)(DISP_HEIGHT_PX - y);
      f->xFrac = 1.0 - (double)x / (double)DISP_WIDTH_PX;
      f->yFrac = 1.0 - (double)y / (double)DISP_HEIGHT_PX;
      break;

    case USBH_WC_ROTATE_270:
      f->x = y;
      f->y = (uint16_t)(DISP_WIDTH_PX - x);
      f->patchWidth  = ph;
      f->patchHeight = pw;
      f->xFrac = (double)y / (double)DISP_HEIGHT_PX;
      f->yFrac = 1.0 - (double)x / (double)DISP_WIDTH_PX;
      break;
  }
}

USBH_StatusTypeDef USBH_HID_WisecocoInit(USBH_HandleTypeDef *phost)
{
  // as far as I can tell these duplicate the non-2 versions of these fields and aren't needed
  UNUSED(prop_x2);
  UNUSED(prop_y2);
  // vendor reports we don't support
  UNUSED(prop_cert_blob);
  UNUSED(prop_vendor_in2);
  UNUSED(prop_vendor_out3);
  UNUSED(prop_vendor_in4);

  HID_HandleTypeDef * const HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  // clear all our state and storage buffers
  memset(&latestData, 0, sizeof(latestData));
  memset(touch_report_data, 0, sizeof(touch_report_data));
  touchReportState = PRS_IDLE;
  memset(touch_rx_report_buf, 0, sizeof(touch_rx_report_buf));
  // TODO do we actually need any of these? I don't think we will ever get these reports
  memset(feature_contact_max_data, 0, sizeof(feature_contact_max_data));
  memset(feature_cert_blob_data, 0, sizeof(feature_cert_blob_data));
  memset(vendor_in_report2_data, 0, sizeof(vendor_in_report2_data));
  memset(vendor_out_report3_data, 0, sizeof(vendor_out_report3_data));
  memset(vendor_in_report4_data, 0, sizeof(vendor_in_report4_data));

  // this is used by the functions in usbh_hid.c which receive data from the USB stack and copy it into our FIFO, so we can't
  // just drop it. For now use 64 bytes, which is the max we can receive from an interrupt transfer (and we check how many
  // actually came in)
  HID_Handle->length = 64;

  // raw report is stored in this buffer before it gets sent to us via the FIFO
  HID_Handle->pData = touch_rx_report_buf;

  if ((HID_QUEUE_SIZE * sizeof(touch_report_data)) > sizeof(phost->device.Data)) {
    return USBH_FAIL;
  }
  else {
    USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(touch_report_data)));
  }

  return USBH_OK;
}

// return the latest? touch report info to be printed above
void USBH_HID_PumpTouchReports(USBH_HandleTypeDef *phost) {
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
  assert(HID_Handle->fifo.buf != NULL);

  if(HID_Handle->pDataLastXferSize == 0) {
    // nothing to do, return
    return;
  }

  // there is fresh data, get it and decode it
  // the demo FIFO stuff is broken in pointless ways, just read pDataLastXferSize bytes out of pData directly
  // we get 116 byte reports split into a 64 byte (max transfer size) + 52 byte. Transfer twice and reassemble them before parsing

  if(touchReportState == PRS_IDLE) {
    // receive the first chunk
    memcpy(&touch_report_data[0], HID_Handle->pData, HID_Handle->pDataLastXferSize);
    touchReportState = PRS_PT2;
  }
  else if(touchReportState == PRS_PT2) {
    memcpy(&touch_report_data[64], HID_Handle->pData, HID_Handle->pDataLastXferSize);
    touchReportState = PRS_READY;
  }

  // mark that we have handled any pending data
  // TODO this should happen via an OS queue or something above this level
  HID_Handle->pDataLastXferSize = 0;

  if(touchReportState != PRS_READY) {
    return;
  }

  // assume we will process this and preemptively mark that we consumed this data and are waiting for the next packet
  touchReportState = PRS_IDLE;

  // confirm this is a supported report by looking at the first byte and the length
  uint8_t const reportId = touch_report_data[0];

  // TODO handle other report types
  if(reportId == 10) {
    // ID 10 is just number of contacts in a single byte
    printf("Rep 10: %lu contacts\n", HID_ReadItem((HID_Report_ItemTypedef*) &prop_contact_count_max, 0));
    // never seen one of these in the wild, so flag it if we do
    assert(0);
    return;
  }
  else if(reportId != 1) {
    // we don't support other report types
    printf("Unknown report %d\n", reportId);
    return;
  }
  // else, report is 1 (the main one)

  // Build the new frame on the stack with no locks held. Once it's fully
  // populated we publish it into latestData under a brief critical section.
  struct USBH_LatestWisecocoData scratch = {0};
  scratch.liveTouches = HID_ReadItem((HID_Report_ItemTypedef*) &prop_contact_count, 0);
  scratch.tipsTouchedDown = 0;

  for(unsigned i = 0; i < scratch.liveTouches; i++) {
    scratch.fingers[i].touching     = HID_ReadItem((HID_Report_ItemTypedef*) &prop_tip[i], 0);
    scratch.fingers[i].id           = HID_ReadItem((HID_Report_ItemTypedef*) &prop_contact_id[i], 0);
    scratch.fingers[i].patchWidth   = HID_ReadItem((HID_Report_ItemTypedef*) &prop_width[i], 0);
    scratch.fingers[i].patchHeight  = HID_ReadItem((HID_Report_ItemTypedef*) &prop_height[i], 0);
    scratch.fingers[i].x            = HID_ReadItem((HID_Report_ItemTypedef*) &prop_x[i], 0);
    scratch.fingers[i].y            = HID_ReadItem((HID_Report_ItemTypedef*) &prop_y[i], 0);

    // apply user-configured rotation; sets x/y/xFrac/yFrac and may swap patch dims
    apply_rotation(&scratch.fingers[i]);

    scratch.fingers[i].touchDuration = HID_ReadItem((HID_Report_ItemTypedef*) &prop_scan_time, 0);
    scratch.tipsTouchedDown += scratch.fingers[i].touching;
  }

  // Publish: atomic struct copy, then signal the consumer.
  taskENTER_CRITICAL();
  latestData = scratch;
  taskEXIT_CRITICAL();
  if (s_newFrameSem != NULL) {
    (void)xSemaphoreGive(s_newFrameSem);
  }
}

