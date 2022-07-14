/*
 * __________________________________________________________________
 *
 * Copyright (C) [2022] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY  AND FITNESS. IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE  FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * __________________________________________________________________
 */

/** @defgroup DriverIcp201xx Icp201xx driver
 *  @brief    High-level driver for Icp201xx devices
 *  @ingroup  DriverIcp201xx
 *  @{
 */

#ifndef _INV_ICP201XX_EXT_H_
#define _INV_ICP201XX_EXT_H_

#include "Icp201xxDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Configure Mode4. Mode4 is user defined mode which allows user to set all parameters like ODR,OSR etc.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] ver Version of ICP201xx 0: for A1 0xb2 for B2.
 * @param[in] pres_osr Over Sampling Ratio (OSR) for pressure measurement.
 * @param[in] temp_osr Over Sampling Ratio (OSR) for temperature measurement.
 * @param[in] odr Output Data Rate (ODR)  .
 * @param[in] HFOSC_on HF oscillator control in between measurements.
 * @param[in] DVDD_on DVDD control in between measurements .
 * @param[in] IIR_filter_en IIR Filter Selection.
 * @param[in] FIR_filter_en FIR Filter Selection.
 * @param[in] IIR_k Pressure IIR filtering to reduce noise effects.
 * @param[in] pres_bs pressure barrel shifter .
 * @param[in] temp_bs temperature barrel shifter. 
 * @param[in] press_gain Pressure gain factor.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_set_mode4_config(inv_icp201xx_t * s,uint8_t ver, uint8_t pres_osr, uint8_t temp_osr, uint16_t odr, uint8_t HFOSC_on,
									uint8_t DVDD_on , uint8_t IIR_filter_en, uint8_t FIR_filter_en, uint16_t IIR_k,
									uint8_t pres_bs,uint8_t temp_bs, uint16_t press_gain);
											

/** @brief Retrieve Mode4 configuration. .
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] ver Version of ICP201xx 0: for A1 0xb2 for B2.
 * @param[out] pointer to pres_osr Over Sampling Ratio (OSR) for pressure measurement.
 * @param[out] pointer to temp_osr Over Sampling Ratio (OSR) for temperature measurement.
 * @param[out] pointer to odr Output Data Rate (ODR)  .
 * @param[out] pointer to HFOSC_on HF oscillator control in between measurements.
 * @param[out] pointer to DVDD_on DVDD control in between measurements .
 * @param[out] pointer to IIR_filter_en IIR Filter Selection.
 * @param[out] pointer to FIR_filter_en FIR Filter Selection.
 * @param[out] pointer to IIR_k Pressure IIR filtering to reduce noise effects.
 * @param[out] pointer to pres_bs pressure barrel shifter.
 * @param[out] pointer to temp_bs temperature barrel shifter .
 * @param[out] pointer to press_gain Pressure gain factor.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_mode4_config(inv_icp201xx_t * s,uint8_t ver, uint8_t *pres_osr, uint8_t *temp_osr, uint16_t *odr, uint8_t *HFOSC_on,
											uint8_t *DVDD_on , uint8_t *IIR_filter_en, uint8_t *FIR_filter_en, uint16_t *IIR_k,
											uint8_t *pres_bs,uint8_t *temp_bs, uint16_t *press_gain);



#ifdef __cplusplus
}
#endif

#endif /* _INV_ICP201XX_EXT_H_ */
