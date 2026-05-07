#include "protocol_task.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include "cobs.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "touch_force.pb.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Worst-case sizes for v1.
 *
 *   Request encoded:           request_id (5) + oneof tag (1) + sub-msg
 *                              length (1) + GetUptimeRequest body (0)
 *                              = 7 bytes typical.
 *   Response encoded:          request_id (5) + oneof tag (1) + sub-msg
 *                              length (1) + GetUptimeResponse uptime_ms
 *                              (5) = 12 bytes typical, ErrorResponse
 *                              with full 64-byte message ~75 bytes.
 *
 * COBS overhead: 1 byte per 254 bytes plus a leading overhead byte.
 * For our sizes that means +2 bytes worst case.
 *
 * 128 is comfortably oversized and easy to reason about. */
#define PROTO_FRAME_MAX_BYTES   128U
#define PROTO_DECODED_MAX_BYTES 128U

/* Read timeout for assembling a single frame. We don't actually need
 * a deadline here -- the loop just blocks on CDC_Read until the host
 * sends something -- so use osWaitForever via the HAL_MAX_DELAY
 * sentinel that CDC_Read recognizes. */
#define PROTO_RX_BLOCK_FOREVER  HAL_MAX_DELAY

/* CDC_Write timeout, milliseconds. Generous; the existing _CDC_Task
 * echo path uses 100ms with no observed problems. */
#define PROTO_TX_TIMEOUT_MS     200U

static uint8_t s_rxFrame[PROTO_FRAME_MAX_BYTES];
static size_t  s_rxFrameLen;
static uint8_t s_decodedBuf[PROTO_DECODED_MAX_BYTES];
static uint8_t s_responseBuf[PROTO_DECODED_MAX_BYTES];
static uint8_t s_txFrame[PROTO_FRAME_MAX_BYTES];

/* Reset the frame accumulator. Called after every successful frame
 * dispatch and after every overflow drop. */
static void rx_reset(void)
{
  s_rxFrameLen = 0U;
}

/* Append one byte to the frame accumulator. Returns false (and
 * resets the accumulator) on overflow. */
static bool rx_append(uint8_t b)
{
  if (s_rxFrameLen >= PROTO_FRAME_MAX_BYTES)
  {
    rx_reset();
    return false;
  }
  s_rxFrame[s_rxFrameLen++] = b;
  return true;
}

/* Build the GetUptime response. */
static void fill_get_uptime_response(touchforce_v1_Response *resp)
{
  resp->which_payload = touchforce_v1_Response_get_uptime_tag;
  resp->payload.get_uptime.uptime_ms = (uint32_t)HAL_GetTick();
}

/* Build a generic error response. msg may be NULL. */
static void fill_error_response(touchforce_v1_Response *resp,
                                uint32_t code, const char *msg)
{
  resp->which_payload = touchforce_v1_Response_error_tag;
  resp->payload.error.code = code;
  resp->payload.error.message[0] = '\0';
  if (msg != NULL)
  {
    /* The .options file bounds .message at 64 bytes. */
    size_t cap = sizeof(resp->payload.error.message);
    strncpy(resp->payload.error.message, msg, cap - 1U);
    resp->payload.error.message[cap - 1U] = '\0';
  }
}

/* Encode resp into s_responseBuf, COBS-encode that into s_txFrame
 * with a trailing 0x00, and CDC_Write it. */
static void send_response(const touchforce_v1_Response *resp)
{
  pb_ostream_t os = pb_ostream_from_buffer(s_responseBuf,
                                           sizeof(s_responseBuf));
  if (!pb_encode(&os, touchforce_v1_Response_fields, resp))
  {
    /* Encoder ran out of buffer or hit a logic bug. Nothing useful
     * we can transmit; drop. */
    return;
  }

  cobs_encode_result enc = cobs_encode(s_txFrame, sizeof(s_txFrame) - 1U,
                                       s_responseBuf, os.bytes_written);
  if (enc.status != COBS_ENCODE_OK)
  {
    return;
  }
  s_txFrame[enc.out_len] = 0x00U;

  (void)CDC_Write(s_txFrame, (uint16_t)(enc.out_len + 1U),
                  PROTO_TX_TIMEOUT_MS);
}

/* Process a single COBS-delimited frame from s_rxFrame. */
static void handle_frame(void)
{
  if (s_rxFrameLen == 0U)
  {
    return;
  }

  /* COBS-decode in place into s_decodedBuf. */
  cobs_decode_result dec = cobs_decode(s_decodedBuf, sizeof(s_decodedBuf),
                                       s_rxFrame, s_rxFrameLen);
  if (dec.status != COBS_DECODE_OK)
  {
    /* Frame was malformed; we don't echo a Response because we have
     * no request_id to echo into it. Just drop. */
    return;
  }

  touchforce_v1_Request req = touchforce_v1_Request_init_zero;
  pb_istream_t is = pb_istream_from_buffer(s_decodedBuf, dec.out_len);
  if (!pb_decode(&is, touchforce_v1_Request_fields, &req))
  {
    return;
  }

  touchforce_v1_Response resp = touchforce_v1_Response_init_zero;
  resp.request_id = req.request_id;

  switch (req.which_payload)
  {
    case touchforce_v1_Request_get_uptime_tag:
      fill_get_uptime_response(&resp);
      break;
    default:
      /* Unknown command tag (host running newer protocol than MCU).
       * Reply with ErrorResponse so the host can correlate. */
      fill_error_response(&resp, 1U, "unknown command");
      break;
  }

  send_response(&resp);
}

void Protocol_RunForever(void)
{
  rx_reset();

  uint8_t chunk[64];

  for (;;)
  {
    size_t n = CDC_Read(chunk, sizeof(chunk), PROTO_RX_BLOCK_FOREVER);
    if (n == 0U)
    {
      continue;
    }

    for (size_t i = 0U; i < n; i++)
    {
      uint8_t b = chunk[i];
      // flag indicating start of a new frame
      if (b == 0x00U)
      {
        handle_frame();
        rx_reset();
      }
      else
      {
        (void)rx_append(b);
      }
    }
  }
}
