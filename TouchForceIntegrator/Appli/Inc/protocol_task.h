#ifndef PROTOCOL_TASK_H
#define PROTOCOL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Run the protocol dispatcher in the calling FreeRTOS task forever.
 * Blocks on CDC_Read, decodes COBS-framed protobuf Requests, and
 * writes COBS-framed protobuf Responses back via CDC_Write.
 *
 * Expects the caller (currently _CDC_Task) to have already waited
 * for USB enumeration before calling.
 */
void Protocol_RunForever(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_TASK_H */
