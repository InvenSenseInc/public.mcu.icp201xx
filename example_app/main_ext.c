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
#include "Icp201xx.h"
#include "Icp201xxDriver.h"
#include "Icp201xxExt.h"
#include "Icp201xxDriverExt.h"
#include "main_ext.h"

/** User configurable MODE4 parameters **/
/* osr_pressure_settings = ( OSR / 2 ^ 5 ) -1 **/
uint8_t icp201xx_mode4_config_press_osr =        0xb1;
uint8_t icp201xx_mode4_config_temp_osr  =        0X0F;
/* odr_setting = ( 8000 / ODR in Hz ) -1  : 25 Hz => ODR setting = 320(0x140) **/
uint16_t icp201xx_mode4_config_odr_setting =      0x140 ;
uint8_t icp201xx_mode4_config_hfosc_en =         0;
uint8_t icp201xx_mode4_config_dvdd_en  =         0;
uint8_t icp201xx_mode4_config_iir_en   =         0;
uint8_t icp201xx_mode4_config_fir_en  =          0;
uint16_t icp201xx_mode4_config_iir_k_factor =     1;

static uint8_t is_mode4_default_config_retrived  = 0;

/* Variables to store default Mode4 config */
static uint8_t m4_default_pres_osr, m4_default_temp_osr,  m4_default_HFOSC_on,	m4_default_DVDD_on , m4_default_IIR_filter_en, m4_default_FIR_filter_en,m4_default_pres_bs,m4_default_temp_bs;
static uint16_t m4_default_odr =0,m4_default_IIR_k = 0, m4_default_press_gain =0;
static uint8_t icp_ver = 0;

int inv_icp201xx_app_pre_start_config(inv_icp201xx_t *s, icp201xx_op_mode_t op_mode,icp201xx_meas_mode_t meas_mode)
{
	int status = INV_ERROR_SUCCESS;
	uint8_t pres_bs,temp_bs,max_idx,max_press_bs;
	uint16_t press_gain = 0;
	uint8_t fir_en = icp201xx_mode4_config_fir_en;
	float icp201xx_mode4_config_press_osr_val; // value of OSR 
	
	if ( op_mode != ICP201XX_OP_MODE4)
		return status;
	if ( !is_mode4_default_config_retrived )
	{
		/* Get current version of ICP201xx as Mode 4 config registers are different for A1 and B2 */
		status = inv_icp201xx_rd_version(&(s->serif),&icp_ver);
		if ( status )
			return status;
		/* One time read Mode4 default config after bootup **/ 
		inv_icp201xx_get_mode4_config(s,icp_ver,&m4_default_pres_osr, &m4_default_temp_osr, &m4_default_odr, &m4_default_HFOSC_on,	&m4_default_DVDD_on , 
										&m4_default_IIR_filter_en, &m4_default_FIR_filter_en, &m4_default_IIR_k,&m4_default_pres_bs,&m4_default_temp_bs,&m4_default_press_gain);
										
		is_mode4_default_config_retrived = 1;
	}
	if ( ICP201XX_MEAS_MODE_FORCED_TRIGGER ==  meas_mode )
		fir_en = 0;

	/** calculate gain factor from default m4 config **/
	
	// Calculate the barrel shifter values and pressure adjustable gain based on
	// the table and formula provided in section BARREL SHIFTER AND ADJUSTABLE GAIN in the App note

	// BS_SHIFT_VAL_PRESS = index, if pres_bs_cond[index+1] < OSR_PRESS <= pres_bs_cond[index]
	
	do{
		/* Look up table to calculate press bs based on press OSR ( refer App note for look up table and formula )
		---------------------------------------------------------------------
		|  BS_SHIFT_VAL_PRESS	  |        Condition for                    |
		|( look up table index)   |       pressure OSR value                |
		------------------------------------For A1 & B2 -------------------
		|       0                |  2^12.5 < OSR Value <= 2^13(8192)        |
		|       1                |  2^12 < OSR Value <= 2^12.5(5792.6)      |
		|       2                |  2^11.5 < OSR Value <= 2^12(4096)        |
		|       3                |  2^11 < OSR Value <= 2^11.5(2896.3)      |
		|       4                |  2^10.5 < OSR Value <= 2^11(2048)        |
		|       5                |  2^10 < OSR Value <= 2^10.5(1448.1)      |
		|       6                |  2^9.5 < OSR Value <= 2^10(1024)         |
		|       7                |  2^9 < OSR Value <= 2^9.5(724)           |
		|       8                |  2^8.5 < OSR Value <= 2^9(512)           |
		------------------------------------- Only For A1 ------------------|
		|       9 - 15           |            reserved                      |
		------------------------------------- Only For B2 -----------------=|
		|       9                |  2^8 < OSR Value <= 2^8.5(362)           |
		|       10               |  2^7.5 < OSR Value <= 2^8(256)           |
		|       11               |  2^7 < OSR Value <= 2^7.5(181)           |
		|       12               |  2^6.5 < OSR Value <= 2^7(128)           |
		|       13               |  2^6 < OSR Value <= 2^6.5(90.5)          |
		|       14               |  2^5.5 < OSR Value <= 2^6(64)            |
		|       15               |  2^5 < OSR Value <= 2^5.5(45.2)          |

		-------------------------------------------------------------------------------------------
		*/
		float fpress_bs_cond[] = {8192, 5792.6, 4096, 2896.3, 2048, 1448.1, 1024, 724, 512, 362, 256, 181, 128, 90.5, 64, 45.2, 32 };
		if (icp_ver == 0 )
		{
			max_idx = 8;
			max_press_bs = 8;
		}
		else
		{
			max_idx = 15;
			max_press_bs = 15;
		}
		/* converting OSR press register value to OSR value to calculate pressure bs using pressure osr value*/
		icp201xx_mode4_config_press_osr_val = (float)((icp201xx_mode4_config_press_osr + 1 ) << 5 );
		for(int i= max_idx; i>=0; i--)
		{
			/* Parse look up table and compare PRESS_OSR reg value for press bs value **/
			if(icp201xx_mode4_config_press_osr_val <= fpress_bs_cond[i])
			{
				pres_bs = i;  /* index of look up table is press bs */
				break;
			}
		}
		
		if(icp201xx_mode4_config_temp_osr == 31)
		{
			if (icp_ver == 0 )
				temp_bs = 6;
			else
				temp_bs = 7;
		}
		else
		{
			if (icp_ver == 0 )
				temp_bs = 8;
			else
				temp_bs = 9;
		}
		
	}while(0);
	
	do{
		float f_curgain;
		do {

			f_curgain = m4_default_press_gain * pow(256,2) / ( (pow((icp201xx_mode4_config_press_osr+1), 2) * pow(2, pres_bs))) ;
			/* if calculated gain is greater than 2.0, bs should be incremented by 1 and gain needs to be recalculated until gain is less than 2.0 */
			/* gain is in Q15 format so comparing with 2 << 15 **/
			if ( f_curgain < (0x02 << 15 ) ) 
			{
				break;
			}
			pres_bs++;

		}while(1);
		
		press_gain = (uint16_t)(f_curgain );
	}while(0);
   
	if ( pres_bs > max_press_bs )
		return INV_ERROR;
		
		
	// set Mode4 config
	status = inv_icp201xx_set_mode4_config(s,icp_ver,
											icp201xx_mode4_config_press_osr,
											icp201xx_mode4_config_temp_osr,
											icp201xx_mode4_config_odr_setting,
											icp201xx_mode4_config_hfosc_en,
											icp201xx_mode4_config_dvdd_en,
											icp201xx_mode4_config_iir_en,
											fir_en,
											icp201xx_mode4_config_iir_k_factor,pres_bs,temp_bs,press_gain);

	return status;
}

int inv_icp201xx_app_is_FIR_Enabled(inv_icp201xx_t	*s, icp201xx_op_mode_t op_mode)
{
	int ret = 1;
	
	if ( op_mode == ICP201XX_OP_MODE4)
	{
		 uint8_t pres_osr, temp_osr,  HFOSC_on,	DVDD_on , IIR_filter_en, FIR_filter_en,pres_bs,temp_bs;
		 uint16_t odr =0,IIR_k = 0, press_gain =0;
		 inv_icp201xx_get_mode4_config(s,icp_ver,&pres_osr, &temp_osr, &odr, &HFOSC_on,	&DVDD_on , &IIR_filter_en, &FIR_filter_en, &IIR_k,&pres_bs,&temp_bs,&press_gain);
		if(!FIR_filter_en)
			ret = 0;
	}
	return ret;
}

