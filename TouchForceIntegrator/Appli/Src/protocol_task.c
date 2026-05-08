#include "protocol_task.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include "haptic_area.h"

#include "cobs.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "touch_force.pb.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* RX (host -> MCU) frames stay small: largest current Request is
 * SetHapticArea at ~80 bytes. 256 keeps headroom for future short
 * commands without paying for big buffers we don't need. */
#define PROTO_RX_FRAME_MAX_BYTES  256U
#define PROTO_RX_DECODED_BYTES    256U

/* TX (MCU -> host) frames must fit GetHapticAreaListResponse with
 * 1024 packed-uint32 ids (~5 KB encoded) plus COBS overhead.
 * 8192 covers 1024 ids comfortably and stays a power of 2. */
#define PROTO_TX_FRAME_MAX_BYTES  8192U
#define PROTO_TX_ENCODED_BYTES    8192U

/* Read timeout for assembling a single frame. */
#define PROTO_RX_BLOCK_FOREVER  HAL_MAX_DELAY

/* Outgoing response timeout. Different from Touch_StreamFrame's
 * 5ms drop-on-busy: a Response is correlated to a Request so
 * the host is actively waiting; we should pay a small wait. */
#define PROTO_TX_TIMEOUT_MS     200U

static uint8_t s_rxFrame[PROTO_RX_FRAME_MAX_BYTES];
static size_t  s_rxFrameLen;
static uint8_t s_decodedBuf[PROTO_RX_DECODED_BYTES];
static uint8_t s_responseBuf[PROTO_TX_ENCODED_BYTES];
static uint8_t s_txFrame[PROTO_TX_FRAME_MAX_BYTES];

static void rx_reset(void)
{
  s_rxFrameLen = 0U;
}

static bool rx_append(uint8_t b)
{
  if (s_rxFrameLen >= PROTO_RX_FRAME_MAX_BYTES)
  {
    rx_reset();
    return false;
  }
  s_rxFrame[s_rxFrameLen++] = b;
  return true;
}

static void fill_get_uptime_response(touchforce_v1_Response *resp)
{
  resp->which_payload = touchforce_v1_Response_get_uptime_tag;
  resp->payload.get_uptime.uptime_ms = (uint32_t)HAL_GetTick();
}

static void fill_get_telemetry_response(touchforce_v1_Response *resp)
{
  resp->which_payload = touchforce_v1_Response_get_telemetry_tag;
  uint32_t sent  = 0U;
  uint32_t fails = 0U;
  Touch_GetTelemetry(&sent, &fails);
  resp->payload.get_telemetry.streaming_events_sent = sent;
  resp->payload.get_telemetry.streaming_tx_fails    = fails;
}

static void fill_error_response(touchforce_v1_Response *resp,
                                uint32_t code, const char *msg)
{
  resp->which_payload = touchforce_v1_Response_error_tag;
  resp->payload.error.code = code;
  resp->payload.error.message[0] = '\0';
  if (msg != NULL)
  {
    size_t cap = sizeof(resp->payload.error.message);
    strncpy(resp->payload.error.message, msg, cap - 1U);
    resp->payload.error.message[cap - 1U] = '\0';
  }
}

static void fill_set_haptic_area_response(const touchforce_v1_Request *req,
                                          touchforce_v1_Response *resp)
{
  /* The wire request always has has_area==true for nanopb sub-msg
   * encoding; we treat a missing area as an invalid-rect failure
   * since the rect would be all zero. Copy by value because
   * HapticArea_Set mutates *areaInOut to fill in the assigned id. */
  touchforce_v1_HapticArea area = req->payload.set_haptic_area.area;
  uint32_t errCode = 0U;
  if (HapticArea_Set(&area, &errCode)) {
    resp->which_payload = touchforce_v1_Response_set_haptic_area_tag;
    resp->payload.set_haptic_area.has_area = true;
    resp->payload.set_haptic_area.area     = area;
  } else {
    fill_error_response(resp, errCode,
                        errCode == HAPTIC_AREA_ERR_TABLE_FULL ? "table full"
                      : errCode == HAPTIC_AREA_ERR_INVALID_RECT ? "invalid rect"
                      : "set failed");
  }
}

static void fill_get_haptic_area_response(const touchforce_v1_Request *req,
                                          touchforce_v1_Response *resp)
{
  touchforce_v1_HapticArea area = touchforce_v1_HapticArea_init_zero;
  if (HapticArea_Get(req->payload.get_haptic_area.id, &area)) {
    resp->which_payload = touchforce_v1_Response_get_haptic_area_tag;
    resp->payload.get_haptic_area.has_area = true;
    resp->payload.get_haptic_area.area     = area;
  } else {
    fill_error_response(resp, HAPTIC_AREA_ERR_NOT_FOUND, "not found");
  }
}

static void fill_get_haptic_area_list_response(touchforce_v1_Response *resp)
{
  resp->which_payload = touchforce_v1_Response_get_haptic_area_list_tag;
  uint32_t cap = (uint32_t)HAPTIC_AREA_MAX_COUNT;
  HapticArea_GetList(resp->payload.get_haptic_area_list.ids, &cap);
  resp->payload.get_haptic_area_list.ids_count = (pb_size_t)cap;
}

static void fill_delete_haptic_area_response(const touchforce_v1_Request *req,
                                             touchforce_v1_Response *resp)
{
  if (HapticArea_Delete(req->payload.delete_haptic_area.id)) {
    /* Empty OkResponse — only the oneof tag matters. */
    resp->which_payload = touchforce_v1_Response_delete_haptic_area_tag;
  } else {
    fill_error_response(resp, HAPTIC_AREA_ERR_NOT_FOUND, "not found");
  }
}

/* Wrap resp in Frame{response: resp}, encode via nanopb,
 * COBS-encode + 0x00 delimiter, CDC_Write. */
static void send_response(const touchforce_v1_Response *resp)
{
  touchforce_v1_Frame frame = touchforce_v1_Frame_init_zero;
  frame.which_kind = touchforce_v1_Frame_response_tag;
  frame.kind.response = *resp;

  pb_ostream_t os = pb_ostream_from_buffer(s_responseBuf,
                                           sizeof(s_responseBuf));
  if (!pb_encode(&os, touchforce_v1_Frame_fields, &frame))
  {
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

/* Process a single COBS-delimited frame from s_rxFrame.
 *
 * v2 wire format: every COBS frame is a Frame{kind} message.
 * For host->MCU traffic we only expect Frame{request}; anything
 * else (Response, Event) is dropped because it has no meaning
 * in this direction. */
static void handle_frame(void)
{
  if (s_rxFrameLen == 0U)
  {
    return;
  }

  cobs_decode_result dec = cobs_decode(s_decodedBuf, sizeof(s_decodedBuf),
                                       s_rxFrame, s_rxFrameLen);
  if (dec.status != COBS_DECODE_OK)
  {
    return;
  }

  touchforce_v1_Frame frame = touchforce_v1_Frame_init_zero;
  pb_istream_t is = pb_istream_from_buffer(s_decodedBuf, dec.out_len);
  if (!pb_decode(&is, touchforce_v1_Frame_fields, &frame))
  {
    return;
  }

  /* We only handle Requests on this side of the wire. */
  if (frame.which_kind != touchforce_v1_Frame_request_tag)
  {
    return;
  }
  const touchforce_v1_Request *req = &frame.kind.request;

  /* SetTouchStreaming is fire-and-forget — apply state and return
   * with no Response. */
  if (req->which_payload == touchforce_v1_Request_set_touch_streaming_tag)
  {
    Touch_SetStreaming(req->payload.set_touch_streaming.enabled);
    return;
  }

  /* Other commands always send a Response (even errors) so the
   * host can correlate by request_id. */
  touchforce_v1_Response resp = touchforce_v1_Response_init_zero;
  resp.request_id = req->request_id;

  switch (req->which_payload)
  {
    case touchforce_v1_Request_get_uptime_tag:
      fill_get_uptime_response(&resp);
      break;
    case touchforce_v1_Request_get_telemetry_tag:
      fill_get_telemetry_response(&resp);
      break;
    case touchforce_v1_Request_set_haptic_area_tag:
      fill_set_haptic_area_response(req, &resp);
      break;
    case touchforce_v1_Request_get_haptic_area_tag:
      fill_get_haptic_area_response(req, &resp);
      break;
    case touchforce_v1_Request_get_haptic_area_list_tag:
      fill_get_haptic_area_list_response(&resp);
      break;
    case touchforce_v1_Request_delete_haptic_area_tag:
      fill_delete_haptic_area_response(req, &resp);
      break;
    case touchforce_v1_Request_delete_all_haptic_areas_tag:
      HapticArea_DeleteAll();
      resp.which_payload = touchforce_v1_Response_delete_all_haptic_areas_tag;
      break;
    case touchforce_v1_Request_set_haptic_area_mode_tag:
      HapticArea_SetMode(req->payload.set_haptic_area_mode.enabled);
      resp.which_payload = touchforce_v1_Response_set_haptic_area_mode_tag;
      resp.payload.set_haptic_area_mode.enabled = HapticArea_GetMode();
      break;
    case touchforce_v1_Request_get_haptic_area_mode_tag:
      resp.which_payload = touchforce_v1_Response_get_haptic_area_mode_tag;
      resp.payload.get_haptic_area_mode.enabled = HapticArea_GetMode();
      break;
    default:
      /* Unknown command tag (host running newer protocol than MCU,
       * or empty oneof / corrupt payload). Reply with ErrorResponse
       * so the host can correlate. */
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
      // 0x00 marks the end of one COBS frame and the start of the next
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
