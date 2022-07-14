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
 *  @brief    Low-level driver for Icp201xx devices
 *  @ingroup  DriverIcp201xx
 *  @{
 */


#include "Icp201xxDefs.h"
#include "Icp201xxSerif.h"
#include "Icp201xxDriver.h"
#include "Icp201xxDriverExt.h"
#include "Icp201xxExtFunc.h"

#include "InvError.h"

/********************************************
Register Name: MODE4_OSR_PRESS
Register Type: READ/WRITE
Register Address: 44 (Decimal); 2C (Hex)
********************************************/
int inv_icp201xx_wr_mode4_osr_press(inv_icp201xx_serif_t  * s, uint8_t ver, uint8_t new_value)
{
	uint8_t reg = MPUREG_A1_MODE4_OSR_PRESS;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_OSR_PRESS;
	}
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
	
}
int inv_icp201xx_rd_mode4_osr_press(inv_icp201xx_serif_t * s, uint8_t ver, uint8_t *value)
{
	uint8_t reg = MPUREG_A1_MODE4_OSR_PRESS;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_OSR_PRESS;
	}
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}


/********************************************
Register Name: MODE4_CONFIG1
Register Type: READ/WRITE
Register Address: 45 (Decimal); 2D (Hex)
********************************************/
int inv_icp201xx_rd_mode4_cfg1(inv_icp201xx_serif_t * s, uint8_t ver, uint8_t *value)
{
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}
int inv_icp201xx_wr_mode4_osr_temp(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MODE4_CONFIG1_OSR_TEMP_MASK) ) | ( new_value ) ;
		
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
		
	return status;
}

int inv_icp201xx_rd_mode4_osr_temp(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;
	
	*value = ( reg_value & BIT_MODE4_CONFIG1_OSR_TEMP_MASK ) ;
	
	return status;
}

int inv_icp201xx_wr_mode4_fir_enable(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_MODE4_CONFIG1_FIR_EN_MASK) ) | ( new_value << BIT_MODE4_CONFIG1_FIR_EN_POS) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}
int inv_icp201xx_rd_mode4_fir_enable(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;
	
	*value = ( reg_value & BIT_MODE4_CONFIG1_FIR_EN_MASK ) >> BIT_MODE4_CONFIG1_FIR_EN_POS ;
	
	return status;
}

int inv_icp201xx_wr_mode4_iir_enable(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MODE4_CONFIG1_IIR_EN_MASK) ) | ( new_value << BIT_MODE4_CONFIG1_IIR_EN_POS) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}
int inv_icp201xx_rd_mode4_iir_enable(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG1;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG1;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;
	
	*value = ( reg_value & BIT_MODE4_CONFIG1_IIR_EN_MASK ) >> BIT_MODE4_CONFIG1_IIR_EN_POS ;
	
	return status;
}


/********************************************
Register Name: MODE4_ODR_LSB
Register Type: READ/WRITE
Register Address: 46 (Decimal); 2E (Hex)
********************************************/
int inv_icp201xx_wr_mode4_odr_lsb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	uint8_t reg = MPUREG_A1_MODE4_ODR_LSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_ODR_LSB;
	}
		
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
 }
int inv_icp201xx_rd_mode4_odr_lsb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	uint8_t reg = MPUREG_A1_MODE4_ODR_LSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_ODR_LSB;
	}
		
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}

/********************************************
Register Name: MODE4_CONFIG2
Register Type: READ/WRITE
Register Address: 47 (Decimal); 2F (Hex)
********************************************/
int inv_icp201xx_wr_mode4_odr_msb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG2;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG2;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MODE4_CONFIG2_ODR_MSB_MASK) ) | ( new_value ) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_mode4_odr_msb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG2;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG2;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;
	
	*value = ( reg_value & BIT_MODE4_CONFIG2_ODR_MSB_MASK ) ;
	
	return status;
}

int inv_icp201xx_wr_mode4_dvdd_on(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG2;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG2;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MODE4_CONFIG2_DVDD_ON_MASK) ) | ( new_value << BIT_MODE4_CONFIG2_DVDD_ON_POS) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}
int inv_icp201xx_rd_mode4_dvdd_on(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG2;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG2;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;
	
	*value = ( reg_value & BIT_MODE4_CONFIG2_DVDD_ON_MASK ) >> BIT_MODE4_CONFIG2_DVDD_ON_POS ;
	
	return status;
}

int inv_icp201xx_wr_mode4_hfosc_on(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG2;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG2;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MODE4_CONFIG2_HFOSC_ON_MASK) ) | ( new_value << BIT_MODE4_CONFIG2_HFOSC_ON_POS) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}
int inv_icp201xx_rd_mode4_hfosc_on(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_CONFIG2;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_CONFIG2;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;
	
	*value = ( reg_value & BIT_MODE4_CONFIG2_HFOSC_ON_MASK ) >> BIT_MODE4_CONFIG2_HFOSC_ON_POS ;
	
	return status;
}
/********************************************
Register Name:
Register Type: READ/WRITE
Register Address: 48(Decimal); 30(Hex)
********************************************/

int inv_icp201xx_wr_mode4_bs_val_press(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_BS_VALUE;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_BS_VALUE;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MODE4_BS_VALUE_PRESS) ) | ( new_value ) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_mode4_bs_val_press(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_BS_VALUE;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_BS_VALUE;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);
	
	if ( status != INV_ERROR_SUCCESS)
	return status;
	
	*value = (reg_value & BIT_MODE4_BS_VALUE_PRESS )  ;
	
	return status;
}

int inv_icp201xx_wr_mode4_bs_val_temp(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_BS_VALUE;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_BS_VALUE;
	}
		
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_MODE4_BS_VALUE_TEMP) ) | ( new_value << 4) ;
	
	status = inv_icp201xx_serif_write_reg(s, reg, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_mode4_bs_val_temp(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t reg = MPUREG_A1_MODE4_BS_VALUE;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_BS_VALUE;
	}
	status = inv_icp201xx_serif_read_reg(s, reg, 1, &reg_value);
	
	if ( status != INV_ERROR_SUCCESS)
	return status;
	
	*value = (reg_value & BIT_MODE4_BS_VALUE_TEMP ) >> 4 ;
	
	return status;
}
/********************************************
Register Name: IIR_K_FACTOR_LSB
Register Type: READ/WRITE
Register Address: 120 (Decimal); 78 (Hex)
********************************************/

int inv_icp201xx_wr_iir_k_factor_lsb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	uint8_t reg = MPUREG_B2_MODE4_IIR_K_FACTOR_LSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_IIR_K_FACTOR_LSB;
	}
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
}

int inv_icp201xx_rd_iir_k_factor_lsb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	uint8_t reg = MPUREG_A1_MODE4_IIR_K_FACTOR_LSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_IIR_K_FACTOR_LSB;
	}
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}

/********************************************
Register Name: IIR_K_FACTOR_MSB
Register Type: READ/WRITE
Register Address: 121 (Decimal); 79 (Hex)
********************************************/

int inv_icp201xx_wr_iir_k_factor_msb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	uint8_t reg = MPUREG_A1_MODE4_IIR_K_FACTOR_MSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_IIR_K_FACTOR_MSB;
	}
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
}

int inv_icp201xx_rd_iir_k_factor_msb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
		uint8_t reg = MPUREG_A1_MODE4_IIR_K_FACTOR_MSB;
		if( ver == 0xb2 )
		{
			reg = MPUREG_B2_MODE4_IIR_K_FACTOR_MSB;
		}
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}

/********************************************
Register Name: MODE0_PRESS_GAIN_FACTOR_LSB
Register Type: READ/WRITE
Register Address: 132 (Decimal); 84 (Hex)
********************************************/
int inv_icp201xx_wr_mode4_press_gain_factor_lsb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	uint8_t reg = MPUREG_A1_MODE4_PRESS_GAIN_FACTOR_LSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_PRESS_GAIN_FACTOR_LSB;
	}
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
}

int inv_icp201xx_rd_mode4_press_gain_factor_lsb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	uint8_t reg = MPUREG_A1_MODE4_PRESS_GAIN_FACTOR_LSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_PRESS_GAIN_FACTOR_LSB;
	}
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}
/********************************************
Register Name: MODE0_PRESS_GAIN_FACTOR_MSB
Register Type: READ/WRITE
Register Address: 133 (Decimal); 85 (Hex)
********************************************/
int inv_icp201xx_wr_mode4_press_gain_factor_msb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value)
{
	uint8_t reg = MPUREG_A1_MODE4_PRESS_GAIN_FACTOR_MSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_PRESS_GAIN_FACTOR_MSB;
	}
		
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
}

int inv_icp201xx_rd_mode4_press_gain_factor_msb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value)
{
	uint8_t reg = MPUREG_A1_MODE4_PRESS_GAIN_FACTOR_MSB;
	if( ver == 0xb2 )
	{
		reg = MPUREG_B2_MODE4_PRESS_GAIN_FACTOR_MSB;
	}
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}



/** @} */

