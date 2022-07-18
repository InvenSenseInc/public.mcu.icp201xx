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

#include "Icp201xx.h"
#include "Icp201xxDriver.h"
#include "Icp201xxExt.h"
#include "Icp201xxDriverExt.h"
#include "Icp201xxExtFunc.h"

int inv_icp201xx_set_mode4_config(inv_icp201xx_t * s, uint8_t ver, uint8_t pres_osr, uint8_t temp_osr, uint16_t odr, uint8_t HFOSC_on, 
									uint8_t DVDD_on , uint8_t IIR_filter_en, uint8_t FIR_filter_en, uint16_t IIR_k,
									uint8_t pres_bs,uint8_t temp_bs, uint16_t press_gain)
{
	int status = INV_ERROR_SUCCESS;

	status |= inv_icp201xx_wr_pow_mode(&(s->serif),ICP201XX_POWER_MODE_ACTIVE);
	inv_icp201xx_sleep_us(2000);

	/* OSR */
	status |= inv_icp201xx_wr_mode4_osr_press(&(s->serif),ver, pres_osr);
	status |= inv_icp201xx_wr_mode4_osr_temp(&(s->serif),ver, temp_osr);
	
	/* ODR */
	status |= inv_icp201xx_wr_mode4_odr_lsb(&(s->serif),ver, (uint8_t)(0xFF & odr));
	status |= inv_icp201xx_wr_mode4_odr_msb(&(s->serif),ver, (uint8_t)(0x1F & (odr >> 8)));
	
	/* IIR */
	status |= inv_icp201xx_wr_mode4_iir_enable(&(s->serif),ver, IIR_filter_en);
	status |= inv_icp201xx_wr_iir_k_factor_lsb(&(s->serif),ver, (uint8_t)(IIR_k & 0xFF ));
	status |= inv_icp201xx_wr_iir_k_factor_msb(&(s->serif), ver,(uint8_t)((IIR_k >>8) & 0xFF ));
	
	/* FIR */
	status |= inv_icp201xx_wr_mode4_fir_enable(&(s->serif),ver, FIR_filter_en);
	
	/* dvdd */
	status |= inv_icp201xx_wr_mode4_dvdd_on(&(s->serif),ver, DVDD_on);
	
	/* dfosc */
	status |= inv_icp201xx_wr_mode4_hfosc_on(&(s->serif),ver, HFOSC_on);
	
	/* Barrel Shifter */
	status |= inv_icp201xx_wr_mode4_bs_val_press(&(s->serif),ver, pres_bs);
	status |= inv_icp201xx_wr_mode4_bs_val_temp(&(s->serif),ver, temp_bs);
	
	/* Pressure gain factor */
	status |= inv_icp201xx_wr_mode4_press_gain_factor_lsb(&(s->serif),ver, (uint8_t)( press_gain & 0xFF ));
	status |= inv_icp201xx_wr_mode4_press_gain_factor_msb(&(s->serif),ver, (uint8_t)( (press_gain >> 8) & 0xFF ));

	return status;
}

int inv_icp201xx_get_mode4_config(inv_icp201xx_t * s,uint8_t ver, uint8_t *pres_osr, uint8_t *temp_osr, uint16_t *odr, uint8_t *HFOSC_on,
											uint8_t *DVDD_on , uint8_t *IIR_filter_en, uint8_t *FIR_filter_en, uint16_t *IIR_k,
											uint8_t *pres_bs,uint8_t *temp_bs, uint16_t *press_gain)
{
	int status = INV_ERROR_SUCCESS;
	uint8_t temp1,temp2;
	/* OSR */
	status |= inv_icp201xx_rd_mode4_osr_press(&(s->serif),ver, pres_osr);
	status |= inv_icp201xx_rd_mode4_osr_temp(&(s->serif),ver, temp_osr);
												
	/* ODR */
	status |= inv_icp201xx_rd_mode4_odr_lsb(&(s->serif),ver, &temp1);
	status |= inv_icp201xx_rd_mode4_odr_msb(&(s->serif),ver, &temp2);
	*odr = (uint16_t) ((temp2 << 8) | temp1 );
										
	/* IIR */
	status |= inv_icp201xx_rd_mode4_iir_enable(&(s->serif),ver, IIR_filter_en);
	status |= inv_icp201xx_rd_iir_k_factor_lsb(&(s->serif),ver, &temp1);
	status |= inv_icp201xx_rd_iir_k_factor_msb(&(s->serif),ver, &temp2);
	*IIR_k = (uint16_t) ((temp2 << 8) | temp1 );
	/* FIR */
	status |= inv_icp201xx_rd_mode4_fir_enable(&(s->serif),ver, FIR_filter_en);
												
	/* dvdd */
	status |= inv_icp201xx_rd_mode4_dvdd_on(&(s->serif),ver, DVDD_on);
												
	/* dfosc */
	status |= inv_icp201xx_rd_mode4_hfosc_on(&(s->serif),ver, HFOSC_on);
	
	/* Barrel Shifter */
	status |= inv_icp201xx_rd_mode4_bs_val_press(&(s->serif),ver, pres_bs);
	status |= inv_icp201xx_rd_mode4_bs_val_temp(&(s->serif),ver, temp_bs);
	
	/* Gain Factor */
	status |= inv_icp201xx_rd_mode4_press_gain_factor_lsb(&(s->serif),ver, &temp1);
	status |= inv_icp201xx_rd_mode4_press_gain_factor_msb(&(s->serif),ver, &temp2);
	*press_gain = (uint16_t) ((temp2 << 8) | temp1 );	
	
												
	return status;
}
											


/** @} */