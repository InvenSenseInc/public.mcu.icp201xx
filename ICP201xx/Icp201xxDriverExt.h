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
#ifndef _INV_ICP201XX_DRIVER_EXT_H_
#define _INV_ICP201XX_DRIVER_EXT_H_

#ifdef __cplusplus
extern "C" {
#endif

/****************** Mode 4 config registers for A1 ****************/
#define MPUREG_A1_MODE4_OSR_PRESS				0x2C
#define MPUREG_A1_MODE4_CONFIG1					0x2D
#define MPUREG_A1_MODE4_ODR_LSB					0x2E
#define MPUREG_A1_MODE4_CONFIG2					0x2F
#define MPUREG_A1_MODE4_BS_VALUE				0x30
#define MPUREG_A1_MODE4_IIR_K_FACTOR_LSB		0x78
#define MPUREG_A1_MODE4_IIR_K_FACTOR_MSB		0x79
#define MPUREG_A1_MODE4_PRESS_GAIN_FACTOR_LSB	0x82
#define MPUREG_A1_MODE4_PRESS_GAIN_FACTOR_MSB	0x83


/****************** Mode 4 config registers for B2 ****************/
#define MPUREG_B2_MODE4_OSR_PRESS				0x3C
#define MPUREG_B2_MODE4_CONFIG1					0x3D
#define MPUREG_B2_MODE4_ODR_LSB					0x3E
#define MPUREG_B2_MODE4_CONFIG2					0x3F
#define MPUREG_B2_MODE4_BS_VALUE				0x40
#define MPUREG_B2_MODE4_IIR_K_FACTOR_LSB		0x88
#define MPUREG_B2_MODE4_IIR_K_FACTOR_MSB		0x89
#define MPUREG_B2_MODE4_PRESS_GAIN_FACTOR_LSB	0x92
#define MPUREG_B2_MODE4_PRESS_GAIN_FACTOR_MSB	0x93


/********************************************
Register Name: MODE4_OSR_PRESS
Register Type: READ/WRITE
Register Address: 44 (Decimal); 2C (Hex)
********************************************/
int inv_icp201xx_wr_mode4_osr_press(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_osr_press(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);

/********************************************
Register Name: MODE4_CONFIG1
Register Type: READ/WRITE
Register Address: 45 (Decimal); 2D (Hex)
********************************************/
#define BIT_MODE4_CONFIG1_OSR_TEMP_MASK         0x1F
#define BIT_MODE4_CONFIG1_FIR_EN_MASK			0x20
#define BIT_MODE4_CONFIG1_IIR_EN_MASK			0x40

#define BIT_MODE4_CONFIG1_FIR_EN_POS			5
#define BIT_MODE4_CONFIG1_IIR_EN_POS			6

int inv_icp201xx_rd_mode4_cfg1(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);
int inv_icp201xx_wr_mode4_osr_temp(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_osr_temp(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);

int inv_icp201xx_wr_mode4_fir_enable(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_fir_enable(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);

int inv_icp201xx_wr_mode4_iir_enable(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_iir_enable(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);

/********************************************
Register Name: MODE4_ODR_LSB
Register Type: READ/WRITE
Register Address: 46 (Decimal); 2E (Hex)
********************************************/
int inv_icp201xx_wr_mode4_odr_lsb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_odr_lsb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);


/********************************************
Register Name: MODE4_CONFIG2
Register Type: READ/WRITE
Register Address: 47 (Decimal); 2F (Hex)
********************************************/
#define BIT_MODE4_CONFIG2_ODR_MSB_MASK           0x1F
#define BIT_MODE4_CONFIG2_DVDD_ON_MASK			 0x20
#define BIT_MODE4_CONFIG2_HFOSC_ON_MASK			 0x40

#define BIT_MODE4_CONFIG2_DVDD_ON_POS			5
#define BIT_MODE4_CONFIG2_HFOSC_ON_POS			6

int inv_icp201xx_wr_mode4_odr_msb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_odr_msb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);

int inv_icp201xx_wr_mode4_dvdd_on(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_dvdd_on(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);

int inv_icp201xx_wr_mode4_hfosc_on(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_hfosc_on(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);


/********************************************
Register Name:
Bank         : Main registers
Register Type: READ/WRITE
Register Address: 48(Decimal); 30(Hex)
********************************************/
#define BIT_MODE4_BS_VALUE_PRESS		0x0F
#define BIT_MODE4_BS_VALUE_TEMP		    0XF0

int inv_icp201xx_wr_mode4_bs_val_press(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_bs_val_press(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t *value);
int inv_icp201xx_wr_mode4_bs_val_temp(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_mode4_bs_val_temp(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t *value);


/********************************************
Register Name: IIR_K_FACTOR_LSB
Register Type: READ/WRITE
Register Address: 120 (Decimal); 78 (Hex)
********************************************/
int inv_icp201xx_wr_iir_k_factor_lsb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_iir_k_factor_lsb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);


/********************************************
Register Name: IIR_K_FACTOR_MSB
Register Type: READ/WRITE
Register Address: 121 (Decimal); 79 (Hex)
********************************************/
int inv_icp201xx_wr_iir_k_factor_msb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);
int inv_icp201xx_rd_iir_k_factor_msb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);



/********************************************
Register Name: MODE0_PRESS_GAIN_FACTOR_LSB
Register Type: READ/WRITE
Register Address: 132 (Decimal); 84 (Hex)
********************************************/
int inv_icp201xx_wr_mode4_press_gain_factor_lsb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);

int inv_icp201xx_rd_mode4_press_gain_factor_lsb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);
/********************************************
Register Name: MODE0_PRESS_GAIN_FACTOR_MSB
Register Type: READ/WRITE
Register Address: 133 (Decimal); 85 (Hex)
********************************************/
int inv_icp201xx_wr_mode4_press_gain_factor_msb(inv_icp201xx_serif_t  * s,uint8_t ver, uint8_t new_value);

int inv_icp201xx_rd_mode4_press_gain_factor_msb(inv_icp201xx_serif_t * s,uint8_t ver, uint8_t *value);


#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _INV_ICP201XX_DRIVER_EXT_H_ */
