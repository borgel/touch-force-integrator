/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

#include "usbh_hid_wisecoco.h"
#include "usbh_hid_parser.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * touch_report_items.c
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

/* ReportID 1: 10 fingers * 7 bytes + 4-byte scan time + 1-byte contact count = 75 bytes. */
// buffer we use to dequeue items from the FIFO into for us to process
uint8_t touch_report_data          [REPORT_ID_OFFSET + 75U];
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
 *     +0  .. +6    Finger  0   (7 bytes)
 *     +7  .. +13   Finger  1
 *     +14 .. +20   Finger  2
 *     +21 .. +27   Finger  3
 *     +28 .. +34   Finger  4
 *     +35 .. +41   Finger  5
 *     +42 .. +48   Finger  6
 *     +49 .. +55   Finger  7
 *     +56 .. +62   Finger  8
 *     +63 .. +69   Finger  9
 *     +70 .. +73   Scan Time       (uint32_t LE, 100 microsecond ticks)
 *     +74          Contact Count   (number of fingers valid this frame)
 *
 *   Each 7-byte finger slot:
 *
 *     byte +0  bit 0       Tip Switch            (0 = no contact, 1 = contact)
 *     byte +0  bit 1       constant pad          (ignore)
 *     byte +0  bits 2..7   Contact Identifier    (0..63, stable per touch)
 *     byte +1              Width                 (contact patch width, 0..255)
 *     byte +2              Height                (contact patch height, 0..255)
 *     byte +3, +4          X coordinate          (uint16 LE, 0..2160)
 *     byte +5, +6          Y coordinate          (uint16 LE, 0..2880)
 *
 *   Note: X and Y are declared with unit exponent -1 and unit 0x11
 *   (SI length, cm), so one logical count = 0.1 mm. Physical ranges
 *   are 1660 (X) and 2940 (Y) in the device's own coordinate space.
 * ========================================================================= */

#define TOUCH_FINGER_COUNT      10U
#define TOUCH_FINGER_STRIDE     7U

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
 * X coordinate (Generic Desktop Usage 0x30): 16-bit little-endian, bytes 3-4.
 * Logical 0..2160, physical 0..1660 (device units), unit = 0.1 mm.
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
 * Y coordinate (Generic Desktop Usage 0x31): 16-bit little-endian, bytes 5-6.
 * Logical 0..2880, physical 0..2940 (device units), unit = 0.1 mm.
 */
static const HID_Report_ItemTypedef prop_y[TOUCH_FINGER_COUNT] =
{
  /* finger 0 */  { .data = FINGER_BYTE(0, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 1 */  { .data = FINGER_BYTE(1, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 2 */  { .data = FINGER_BYTE(2, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 3 */  { .data = FINGER_BYTE(3, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 4 */  { .data = FINGER_BYTE(4, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 5 */  { .data = FINGER_BYTE(5, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 6 */  { .data = FINGER_BYTE(6, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 7 */  { .data = FINGER_BYTE(7, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 8 */  { .data = FINGER_BYTE(8, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
  /* finger 9 */  { .data = FINGER_BYTE(9, 5), .size = 16, .shift = 0, .count = 0, .sign = 0, .logical_min = 0, .logical_max = 2880, .physical_min = 0, .physical_max = 2940, .resolution = 1 },
};

/*
 * Scan Time (Digitizer Usage 0x56): 32-bit counter in 100 us units since
 * an arbitrary start, wrapping at 0x7FFFFFFF. Lives at byte offset 70 of
 * the finger block.
 */
static const HID_Report_ItemTypedef prop_scan_time =
{
  .data         = touch_report_data + REPORT_ID_OFFSET + 70U,
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
 * slots contain valid data in this frame. Lives at byte 74.
 */
static const HID_Report_ItemTypedef prop_contact_count =
{
  .data         = touch_report_data + REPORT_ID_OFFSET + 74U,
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


// hold the most recent update state data we've gotten from the device
static struct USBH_LatestWisecocoData latestData;

static USBH_StatusTypeDef tryGetReport(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef USBH_HID_WisecocoInit(USBH_HandleTypeDef *phost)
{
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

  // FIXME I think this is just for us to use to filter later, and we need to be smarter than that, so drop it?
  // this is used by the functions in usbh_hid.c which receive data from the USB stack and copy it into our FIFO, so we can't
  // just drop it (yet)
  HID_Handle->length = 64; //75; //52; //(uint16_t)sizeof(touch_report_data);

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
bool USBH_HID_GetTouchReport(USBH_HandleTypeDef *phost, struct USBH_LatestWisecocoData * const newReport) {
  // try to decode a new packet if there is one
  // return data if it was just decoded
  if(tryGetReport(phost) == USBH_OK) {
    // TODO unpack fields
    newReport->x = 5;
    return true;
  }
  return false;
}

static USBH_StatusTypeDef tryGetReport(USBH_HandleTypeDef *phost) {
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
  assert(HID_Handle->fifo.buf != NULL);

  // the demo FIFO stuff is broken in pointless ways, just read pDataLastXferSize bytes out of pData directly
  int const packetLen = HID_Handle->pDataLastXferSize;
  if(packetLen == 0) {
    // nothing to do, return
    return USBH_FAIL;
  }

  // FIXME rm
  printf("bytes %d sm %d\n", packetLen, touchReportState);

  // there is fresh data, get it and decode it
  // we get 116 byte reports split into a 64 byte (max transfer size) + 52 byte. Transfer twice and reassemble them before parsing

  if(touchReportState == PRS_IDLE) {
    // receive the first chunk
    memcpy(&touch_report_data[0], HID_Handle->pData, packetLen);
    touchReportState = PRS_PT2;
  }
  else if(touchReportState == PRS_PT2) {
    memcpy(&touch_report_data[64], HID_Handle->pData, packetLen);
    touchReportState = PRS_READY;
  }

  // mark that we have handled any pending data
  // TODO this should happen via an OS queue or something
  HID_Handle->pDataLastXferSize = 0;

  if(touchReportState != PRS_READY) {
    return USBH_FAIL;
  }

  printf("Got full packet\n");

  // mark (for ourselves) that we consumed this data and are waiting for the next packet
  // TODO rm this?
  touchReportState = PRS_IDLE;


  printf("%0d - ", packetLen);
  for(int i = 0; i < packetLen; i++) {
    printf("0x%02x ", HID_Handle->pData[i]);
  }
  printf("\n");

  return USBH_OK;


  /*
  int fifoPacketLen = USBH_HID_FifoRead(&HID_Handle->fifo, touch_report_data, HID_Handle->length);
  if(fifoPacketLen != 64) {
    // if we didn't get the expected transfer size, return, which will retry and resync us
    return USBH_FAIL;
  }
  fifoPacketLen = USBH_HID_FifoRead(&HID_Handle->fifo, &touch_report_data[fifoPacketLen], HID_Handle->length);
  if(fifoPacketLen != 52) {
    return USBH_FAIL;
  }
  fifoPacketLen = 64 + 52;
  */


  // FIXME rm
  int fifoPacketLen = 0;
  printf("%0d - ", fifoPacketLen);
  for(int i = 0; i < fifoPacketLen; i++) {
    printf("0x%02x ", touch_report_data[i]);
  }
  printf("\n");
  return USBH_OK;

  // FIXME rm
  printf("\\/-----\n");

  // confirm this is a supported report by looking at the first byte and the length
  uint8_t const reportId = touch_report_data[0];

  // FIXME rm
  printf("rep %d, len %d\n", reportId, fifoPacketLen);

  // TODO handle other report types
  if(reportId == 10) {
    // ID 10 is just number of contacts in a single byte
    printf("Rep 10: %u contacts\n", touch_report_data[1]);
    return USBH_OK;
  }
  else if(reportId != 1) {
    // we don't support these reports
    printf("X Rep %d\n", reportId);
    return USBH_FAIL;
  }
  // else, report is 1 (the main one)

  // TODO decode into real internal storage struct
  printf("total size %d\n", HID_Handle->length);
  int const touchCount = HID_ReadItem((HID_Report_ItemTypedef*) &prop_contact_count, 0);
  printf("Count %d\n", touchCount);
  for(int i = 0; i < TOUCH_FINGER_COUNT; i++) {

  }
  int finger0touch = HID_ReadItem((HID_Report_ItemTypedef *) &prop_tip[0], 0);

  printf("finger 0 touch: %d\n", finger0touch);

  return USBH_OK;
}

