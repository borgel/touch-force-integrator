/**
  ******************************************************************************
  * @file    usbh_hid_parser.c
  * @author  MCD Application Team
  * @brief   This file is the HID Layer Handlers for USB Host HID class.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid_parser.h"


/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_CLASS
  * @{
  */

/** @addtogroup USBH_HID_CLASS
  * @{
  */

/** @defgroup USBH_HID_PARSER
  * @brief    This file includes HID parsers for USB Host HID class.
  * @{
  */

/** @defgroup USBH_HID_PARSER_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_HID_PARSER_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_HID_PARSER_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_HID_PARSER_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */


/** @defgroup USBH_HID_PARSER_Private_Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup USBH_HID_PARSER_Private_Functions
  * @{
  */

/**
  * @brief  HID_ReadItem
  *         The function read a report item.
  * @param  ri: report item
  * @param  ndx: report index
  * @retval status (0 : fail / otherwise: item value)
  */
uint32_t HID_ReadItem(HID_Report_ItemTypedef *ri, uint8_t ndx)
{
  uint32_t val = 0U;
  uint32_t x;
  uint32_t bofs;
  uint32_t n_bytes;
  uint32_t size_mask;
  uint8_t *data = ri->data;
  uint8_t shift = ri->shift;

  /* Reject degenerate field widths; val is uint32_t, so >32 bits can't fit. */
  if ((ri->size == 0U) || (ri->size > 32U))
  {
    return (0U);
  }

  /* If this is an array item, offset into the packed bit-array by ndx. */
  if (ri->count > 0U)
  {
    if (ri->count <= ndx)
    {
      return (0U);
    }

    bofs = (uint32_t)ndx * ri->size;
    bofs += shift;
    data += bofs / 8U;
    shift = (uint8_t)(bofs % 8U);
  }

  /* Number of bytes covering (shift + size) bits, capped at sizeof(val). */
  n_bytes = ((uint32_t)shift + ri->size + 7U) / 8U;
  if (n_bytes > 4U)
  {
    n_bytes = 4U;
  }

  /* Read bytes in little-endian order, OR-ing each into the accumulator and
     advancing the pointer. (The original code wrote val = ... and never
     advanced data, so multi-byte reads returned only the last byte's bits
     shifted into the top lane.) */
  for (x = 0U; x < n_bytes; x++)
  {
    val |= (uint32_t)(*data) << (x * 8U);
    data++;
  }

  /* Mask to field width. Guard against (1 << 32) which is UB. */
  size_mask = (ri->size >= 32U) ? 0xFFFFFFFFU
                                : (((uint32_t)1U << ri->size) - 1U);
  val = (val >> shift) & size_mask;

  /* Sign-extend BEFORE the range check so signed fields whose logical_min
     is negative (stored as its sign-extended uint32 bit pattern) compare
     correctly. Skip extension when size == 32 — the value already fills val. */
  if ((ri->sign != 0U) && (ri->size < 32U)
      && ((val & ((uint32_t)1U << (ri->size - 1U))) != 0U))
  {
    val |= ~size_mask;
  }

  /* Range check, signed or unsigned depending on ri->sign. */
  if (ri->sign != 0U)
  {
    int32_t sval = (int32_t)val;
    int32_t smin = (int32_t)ri->logical_min;
    int32_t smax = (int32_t)ri->logical_max;
    if ((sval < smin) || (sval > smax))
    {
      return (0U);
    }
  }
  else
  {
    if ((val < ri->logical_min) || (val > ri->logical_max))
    {
      return (0U);
    }
  }

  /* Scale by resolution. Two's-complement multiplication makes the
     unsigned product identical to the signed product for any operand
     signs, so one code path covers both. */
  if (ri->resolution == 1U)
  {
    return (val);
  }
  return (val * ri->resolution);
}

/**
  * @brief  HID_WriteItem
  *         The function write a report item.
  * @param  ri: report item
  * @param  ndx: report index
  * @retval status (1: fail/ 0 : Ok)
  */
uint32_t HID_WriteItem(HID_Report_ItemTypedef *ri, uint32_t value, uint8_t ndx)
{
  uint32_t x;
  uint32_t field_mask;
  uint32_t shifted_val;
  uint32_t shifted_mask;
  uint32_t bofs;
  uint32_t n_bytes;
  uint8_t *data = ri->data;
  uint8_t shift = ri->shift;

  /* Reject degenerate field widths; value is uint32_t, so >32 bits can't fit. */
  if ((ri->size == 0U) || (ri->size > 32U))
  {
    return (1U);
  }

  /* Range check input as a physical value, signed-aware. */
  if (ri->sign != 0U)
  {
    int32_t sval = (int32_t)value;
    int32_t smin = (int32_t)ri->physical_min;
    int32_t smax = (int32_t)ri->physical_max;
    if ((sval < smin) || (sval > smax))
    {
      return (1U);
    }
  }
  else
  {
    if ((value < ri->physical_min) || (value > ri->physical_max))
    {
      return (1U);
    }
  }

  /* If this is an array item, offset into the packed bit-array by ndx.
     (The original code had the predicate inverted and also never used the
     resulting `data` offset, so every array write silently targeted
     element 0 or returned "success" without writing.) */
  if (ri->count > 0U)
  {
    if (ri->count <= ndx)
    {
      return (1U);
    }

    bofs = (uint32_t)ndx * ri->size;
    bofs += shift;
    data += bofs / 8U;
    shift = (uint8_t)(bofs % 8U);
  }

  /* Convert physical value to logical value, preserving signedness. */
  if (ri->resolution != 1U)
  {
    if (ri->sign != 0U)
    {
      value = (uint32_t)((int32_t)value / (int32_t)ri->resolution);
    }
    else
    {
      value = value / ri->resolution;
    }
  }

  /* Compute field mask, guarding against (1 << 32) UB. */
  field_mask = (ri->size >= 32U) ? 0xFFFFFFFFU
                                 : (((uint32_t)1U << ri->size) - 1U);

  /* Position both the value and the mask into their bit slots within the
     output word. The original code shifted only the value, not the mask,
     so the per-byte merge read-modify-wrote the wrong bits whenever shift
     was non-zero (e.g. Contact Identifier at shift 2). */
  shifted_val  = (value & field_mask) << shift;
  shifted_mask = field_mask << shift;

  /* Bytes covering (shift + size) bits, capped at sizeof(uint32_t). */
  n_bytes = ((uint32_t)shift + ri->size + 7U) / 8U;
  if (n_bytes > 4U)
  {
    n_bytes = 4U;
  }

  /* Read-modify-write each byte through `data` (not `ri->data`, which
     would ignore the array offset): preserve bits outside the field,
     overwrite bits inside it. */
  for (x = 0U; x < n_bytes; x++)
  {
    uint8_t byte_mask = (uint8_t)(shifted_mask >> (x * 8U));
    uint8_t byte_val  = (uint8_t)(shifted_val  >> (x * 8U));
    *(data + x) = (uint8_t)((*(data + x) & (uint8_t)(~byte_mask))
                            | (byte_val & byte_mask));
  }

  return (0U);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


/**
  * @}
  */

