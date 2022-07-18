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


/* InvenSense drivers and utils */

#include "Message.h"
#include "ErrorHelper.h"
#include "Icp201xx.h"
#include "Icp201xxExtFunc.h"
#include "Icp201xxDriver.h"
#include "mcu_host_if.h"
#include "main.h"
#include "main_ext.h"

/** Default run mode is set to Interrupt mode with FIFO overflow/underflow interrupts enabled **/
uint8_t icp201xx_default_run_mode =									ICP201XX_RUN_MODE_INTERRUPT_FIFO_WATERMARK;
/* Default FIFO read out mode **/
icp201xx_FIFO_readout_mode_t icp201xx_default_fifo_readout_mode =	ICP201XX_FIFO_READOUT_MODE_PRES_TEMP;

icp201xx_op_mode_t icp201xx_default_op_mode =						ICP201XX_OP_MODE2;

/* Default measurement mode ( Continuous / Trigger )*/
icp201xx_meas_mode_t icp201xx_default_meas_mode =					ICP201XX_MEAS_MODE_CONTINUOUS;


uint8_t icp201xx_cur_int_mask   = ICP201XX_INT_MASK_FIFO_WMK_HIGH | ICP201XX_INT_MASK_FIFO_WMK_LOW ;


/* In Trigger measurement mode, time between triggers sent to ICP201xx in secs **/
uint8_t icp201xx_trigger_mode_tbw_triggers_in_sec = 20;

#define FIFO_DATA_BUFFER_SIZE		200
#define PRESS_TEMP_DATA_BUFFER_SIZE 20

/* extern and global variables */

uint8_t fifo_data[FIFO_DATA_BUFFER_SIZE] = {0};
int32_t data_temp[PRESS_TEMP_DATA_BUFFER_SIZE],data_press[PRESS_TEMP_DATA_BUFFER_SIZE];
inv_icp201xx_t			icp_device = {0,0};
inv_icp201xx_serif_t	icp201xx_serif;
volatile uint32_t		irq_start_pressure_capture;
volatile uint32_t		irq_host_command = 0;
volatile uint8_t		int_status2 = 0;
volatile uint8_t        is_sw0_button_pressed = 0;
extern uint8_t is_valid_UsrCmd ;

/*!
* @brief	Sensor general interrupt handler, calls specific handlers.
*
*
*/

void display_press_temp(uint8_t fifo_packets)
{
	uint8_t i;
	float pres = 0,temp = 0;
	for ( i = 0 ; i < fifo_packets ; i++)
	{

		if ( icp201xx_default_fifo_readout_mode != ICP201XX_FIFO_READOUT_MODE_TEMP_ONLY )
		{
			/** P = (POUT/2^17)*40kPa + 70kPa **/
			if (data_press[i] & 0x080000 )
			data_press[i] |= 0xFFF00000;
			
			pres = ((float)(data_press[i]) *40 /131072) +70;
		}
		if ( icp201xx_default_fifo_readout_mode != ICP201XX_FIFO_READOUT_MODE_PRES_ONLY )
		{
			/* T = (TOUT/2^18)*65C + 25C */
			if (data_temp[i] & 0x080000 )
			data_temp[i] |= 0xFFF00000;
			
			temp = ((float)( data_temp[i] )*65 /262144 ) + 25;
		}
		INV_MSG(INV_MSG_LEVEL_INFO, "%.3f,%.3f",pres,temp);

	}
}

static void inv_external_interrupt_handler(void) {
	irq_start_pressure_capture = 1;
	
	inv_icp201xx_get_int_status(&icp_device,&int_status2);
	inv_icp201xx_clear_int_status(&icp_device,int_status2);
}

void inv_icp201xx_sleep_us(int us) {
	
	//delay_us(us);
}

static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) 
{
	/* Implement Reg Read */
}

static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) 
{
	/* Implement reg write   */
}

/* ICP201xx warm up. 
 * If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values. 
 * Therefore the first 14 pressure output values are discarded.
 **/
static void inv_icp201xx_app_warmup(icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode)
{
	volatile uint8_t fifo_packets = 0;
	uint8_t fifo_packets_to_skip = 14;
	if ( meas_mode == ICP201XX_MEAS_MODE_FORCED_TRIGGER )
	{
		/** In trigger mode, trigger and skip one measurement **/
		fifo_packets_to_skip = 1;
		inv_icp201xx_trigger_meas(&icp_device);
	}else if (!inv_icp201xx_app_is_FIR_Enabled(&icp_device,op_mode))
		fifo_packets_to_skip = 1;
		
	INV_MSG(INV_MSG_LEVEL_INFO, "Ignoring initial %d samples...",fifo_packets_to_skip);
	do{
		fifo_packets = 0;
		if ( !inv_icp201xx_get_fifo_count(&icp_device,&fifo_packets) && ( fifo_packets >= fifo_packets_to_skip ) )	{
			uint8_t i_status = 0;
			inv_icp201xx_flush_fifo(&icp_device);
			
			/* TO_DO :  Disable IRQ here*/ 
			irq_start_pressure_capture = 0;
			inv_icp201xx_get_int_status(&icp_device,&i_status);
			if ( i_status )
				inv_icp201xx_clear_int_status(&icp_device,i_status);
			/* TO_DO :  Enable IRQ here*/
			break;
		}
		delay_us(2);
	} while (1);
	

	INV_MSG(INV_MSG_LEVEL_INFO, "Settling done.");
}

void inv_run_icp201xx_in_polling(icp201xx_op_mode_t op_mode,uint32_t run_time_in_sec)
{
	uint8_t fifo_packets = 0;
	uint32_t start_ticks = timer_ticks;
	uint32_t timestamp = 0;
	int status;
	
	status = inv_icp201xx_soft_reset(&icp_device);
	if ( status )
		INV_MSG(INV_MSG_LEVEL_ERROR,"Soft Reset Error %d",status);
		
	if ( inv_icp201xx_app_pre_start_config(&icp_device,op_mode,ICP201XX_MEAS_MODE_CONTINUOUS) )
		INV_MSG(INV_MSG_LEVEL_ERROR,"ICP201xx Pre start config failed ");
	/* Configure for polling **/
	inv_icp201xx_set_fifo_notification_config(&icp_device, 0 ,0,0);
	
	#if ( EN_PRESS_ABS_INT_TEST == 1)
	/** Absolute pressure Interrupt Test: Test to verify if is triggered on pressure overruns/underruns set abs pressure value **/
	inv_icp201xx_set_press_notification_config(&icp_device,ICP201XX_INT_MASK_PRESS_ABS,PRESS_ABS_VALUE,0); 
	INV_MSG(INV_MSG_LEVEL_INFO,"### Pressure Abs interrupt enabled Pre Abs Value:0x%x ###",PRESS_ABS_VALUE);
	#endif
	#if (EN_PRESS_DELTA_INT_TEST == 1)
	/** Delta pressure interrupt test: Test to verify if int is triggered on diff between 2 consecutive pressure values exceeds	set delta value **/
	inv_icp201xx_set_press_notification_config(&icp_device,ICP201XX_INT_MASK_PRESS_DELTA,0,PRESS_DELTA_VALUE);
	INV_MSG(INV_MSG_LEVEL_INFO,"### Pressure Delta interrupt enabled Pre Delta Value:0x%x ###",PRESS_DELTA_VALUE);
	#endif
	
	status = inv_icp201xx_config(&icp_device,op_mode,icp201xx_default_fifo_readout_mode);
	if ( status ) {
		INV_MSG(INV_MSG_LEVEL_ERROR,"ICP201xx config to run %d mode Error %d",op_mode,status);
		return ;
	}
	
	INV_MSG(INV_MSG_LEVEL_INFO,"### Starting in Polling Mode for %d sec Op Mode:%d ###",run_time_in_sec,op_mode);
	inv_icp201xx_app_warmup(op_mode,ICP201XX_MEAS_MODE_CONTINUOUS);
	while (1) {
		delay_us(5);

		/** Read measurements count in FIFO **/
		if ( inv_icp201xx_get_fifo_count(&icp_device,&fifo_packets) )
			continue;
	
		if ( fifo_packets >= POLLING_MODE_FIFO_THRESOLD || (irq_start_pressure_capture == 1))
		{
			if (irq_start_pressure_capture == 1)
			{
				if ( int_status2) {
					INV_MSG(INV_MSG_LEVEL_INFO, " int received status [0x%x] , fifoCnt:%d",int_status2,fifo_packets);
				}
				
				/* TO_DO :  Disable IRQ here*/  
				irq_start_pressure_capture = 0;
				/* TO_DO :  Enable IRQ here*/
				if ( fifo_packets < 10 ) 
					continue;
			}

			/* TO_DO :  Disable IRQ here*/  
			inv_icp201xx_get_fifo_data(&icp_device,fifo_packets,fifo_data);
			/* Enable IRQ */
			#if(SERIF_TYPE_I2C == 1)
			/* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
			do{
				uint8_t dummy_reg = 0;
				inv_icp201xx_rd_reg(&icp_device,0x00,&dummy_reg);
			}while(0);
			#endif
			inv_icp201xx_process_raw_data(&icp_device,fifo_packets,fifo_data,data_press,data_temp);

			INV_MSG(INV_MSG_LEVEL_INFO, "FIFO_Cnt[%d]",fifo_packets);
			display_press_temp(fifo_packets);
			
		}

	} // end of while 1
	inv_icp201xx_soft_reset(&icp_device);
	INV_MSG(INV_MSG_LEVEL_INFO,"Polling  Mode Done");
}

void inv_run_icp201xx_in_interrupt_mode(icp201xx_op_mode_t op_mode,uint32_t run_time_in_sec)
{
	uint8_t fifo_packets = 0;
	uint32_t start_ticks = timer_ticks;
	uint32_t timestamp = 0;
	volatile uint8_t temp_int_status = 0;
	int status;
	
	status = inv_icp201xx_soft_reset(&icp_device);
	if ( status )
		INV_MSG(INV_MSG_LEVEL_ERROR,"Soft Reset Error %d",status);
		
	if  ( inv_icp201xx_app_pre_start_config(&icp_device,op_mode,ICP201XX_MEAS_MODE_CONTINUOUS) )
		INV_MSG(INV_MSG_LEVEL_ERROR,"ICP201xx Pre start config failed ");
	
	INV_MSG(INV_MSG_LEVEL_INFO,"### Starting in Interrupt Mode IntMask:0x%x for %d sec Op Mode:%d ###",icp201xx_cur_int_mask,run_time_in_sec,op_mode);
	inv_icp201xx_set_fifo_notification_config(&icp_device, icp201xx_cur_int_mask  ,FIFO_WATERMARK_HIGH_VALUE,FIFO_WATERMARK_LOW_VALUE);


	status = inv_icp201xx_config(&icp_device,op_mode,icp201xx_default_fifo_readout_mode);
	if ( status ) {
		INV_MSG(INV_MSG_LEVEL_ERROR,"ICP201xx config to run in op mode %d Error: %d",op_mode,status);
		return ;
	}
	
	inv_icp201xx_app_warmup(op_mode,ICP201XX_MEAS_MODE_CONTINUOUS);

	while (1) {
		/************************** Interrupt based ******************************/
		InvScheduler_dispatchTasks(&scheduler);
		
	
		if (irq_start_pressure_capture == 1) {
			/* TO_DO :  Disable IRQ here*/  
			temp_int_status = int_status2;
			irq_start_pressure_capture = 0;
			/* TO_DO :  Enable IRQ here*/
			
			
			if ( !temp_int_status ){
				INV_MSG(INV_MSG_LEVEL_INFO, "Error Unexpected Interrupt is received IntStatus:0x%x",temp_int_status);
				continue;
			}
			
			while ( inv_icp201xx_get_fifo_count(&icp_device,&fifo_packets) ) 
				delay_us(1);

			INV_MSG(INV_MSG_LEVEL_INFO, "Int_Status [0x%x] FIFO_Cnt[%d]",temp_int_status,fifo_packets);
			if ( fifo_packets > 16 || fifo_packets == 0)
			{
				inv_icp201xx_flush_fifo(&icp_device);
			}else if ( temp_int_status & ( ICP201XX_INT_STATUS_PRESS_DELTA |ICP201XX_INT_STATUS_PRESS_ABS |ICP201XX_INT_STATUS_FIFO_WMK_HIGH |ICP201XX_INT_STATUS_FIFO_OVER_FLOW ))
			{
				/* Avoiding reading FIFO Underflow and FIFO Low watermark interrupt */
				/* Disable IRQ */ 
				inv_icp201xx_get_fifo_data(&icp_device,fifo_packets,fifo_data);
				/* TO_DO :  Enable IRQ here*/
				inv_icp201xx_process_raw_data(&icp_device,fifo_packets,fifo_data,data_press,data_temp);
				display_press_temp(fifo_packets);

			}
			#if(SERIF_TYPE_I2C == 1)
			/* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
			do{
				uint8_t dummy_reg = 0;
				inv_icp201xx_rd_reg(&icp_device,0x00,&dummy_reg);
			}while(0);
			#endif
			
		} 
	}
	inv_icp201xx_soft_reset(&icp_device);

	INV_MSG(INV_MSG_LEVEL_INFO,"Interrupt Mode Done");
}

void inv_run_icp201xx_in_trigger_mode(uint32_t run_time_in_sec)
{
	int status;
	uint32_t start_ticks = timer_ticks;
	uint8_t fifo_packets = 0;
	uint32_t timestamp = 0,trigger_time = 0;
	
	status = inv_icp201xx_soft_reset(&icp_device);
	if ( status )
		INV_MSG(INV_MSG_LEVEL_ERROR,"Soft Reset Error %d",status);

	inv_icp201xx_adv_config(&icp_device,ICP201XX_OP_MODE4,ICP201XX_MEAS_MODE_FORCED_TRIGGER,ICP201XX_FORCE_MEAS_STANDBY,ICP201XX_POWER_MODE_NORMAL,icp201xx_default_fifo_readout_mode);
	INV_MSG(INV_MSG_LEVEL_INFO,"### Starting in Trigger Mode, Op Mode:4 ###");
	
	inv_icp201xx_app_warmup(ICP201XX_OP_MODE4,ICP201XX_MEAS_MODE_FORCED_TRIGGER);
	while(1) {
		/* TO_DO : Set isTriggerMeasNow when ever a measurement needs to be triggred */		
		if ( isTriggerMeasNow )
		{
			inv_icp201xx_trigger_meas(&icp_device);
			
			/* Loop for 1 measurement */
			do {
				fifo_packets = 0;
				// keep polling for fifo size
				if (inv_icp201xx_get_fifo_count(&icp_device,&fifo_packets) )
					continue;
			
				if(fifo_packets > 0 ) {
					/* TO_DO :  Disable IRQ here*/  
					inv_icp201xx_get_fifo_data(&icp_device,fifo_packets,fifo_data);
					/* TO_DO :  Enable IRQ here*/
					inv_icp201xx_process_raw_data(&icp_device,fifo_packets,fifo_data,data_press,data_temp);
					display_press_temp(fifo_packets);
					break;
				}
				delay_us(1);
			}while(1);
			#if(SERIF_TYPE_I2C == 1)
			/* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
			do{
				uint8_t dummy_reg = 0;
				inv_icp201xx_rd_reg(&icp_device,0x00,&dummy_reg);
			}while(0);
			#endif
		}
		delay_us(5); 
		
	} // End of while(1)
	inv_icp201xx_soft_reset(&icp_device);
}

int main (void) {
	
	int32_t	 rc = 0;
	int status;
	uint8_t device_id = 0,version = 0;
	
	/* 
	   - Initialize board
	   - Init Systic
	   - Init Logging 
	   - Init serial interface. 
	   - update icp201xx_serif.read_reg and icp201xx_serif.write_reg with bsp function to read and write registers.
	 */
  
	INV_MSG(INV_MSG_LEVEL_INFO, "##########################");
	INV_MSG(INV_MSG_LEVEL_INFO, "     ICP201xx example     ");
	INV_MSG(INV_MSG_LEVEL_INFO, "##########################");
	
	/*
	 * Reset pressure sensor driver states
	 */
	status = inv_icp201xx_init(&icp_device,&icp201xx_serif);
	if ( status )
		INV_MSG(INV_MSG_LEVEL_ERROR, " ICP201xx Init failed %d",status);
	
	inv_icp201xx_soft_reset(&icp_device);
	if ( status )
		INV_MSG(INV_MSG_LEVEL_ERROR,"Soft Reset Error %d",status);
	
	/** Retrive and validate who_am_i **/
	status =  inv_icp201xx_get_devid_version(&icp_device,&device_id,&version);
	if (status) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Failed to retrieve Device ID or Wrong WHO_Device ID received :0x%x",device_id);
		while(1);
	}
	INV_MSG(INV_MSG_LEVEL_INFO, " Received Device ID: 0x%x. Version:0x%x Success",device_id,version);

	/** Boot up OTP config **/
	status = inv_icp201xx_OTP_bootup_cfg(&icp_device);
	if ( status )
		INV_MSG(INV_MSG_LEVEL_ERROR, " Bootup OTP Config failed ");
	
	if ( icp201xx_default_meas_mode == ICP201XX_MEAS_MODE_FORCED_TRIGGER ) {
		inv_run_icp201xx_in_trigger_mode(0);
	}
	else{
		/* By default start measurements */	
		if ( ICP201XX_RUN_MODE_INTERRUPT_FIFO_OVERFLOW == icp201xx_default_run_mode)
		{
			icp201xx_cur_int_mask   = ICP201XX_INT_MASK_FIFO_OVER_FLOW | ICP201XX_INT_MASK_FIFO_UNDER_FLOW; 
			inv_run_icp201xx_in_interrupt_mode(icp201xx_default_op_mode,0);
		}
		else if ( ICP201XX_RUN_MODE_INTERRUPT_FIFO_WATERMARK == icp201xx_default_run_mode)
		{
			icp201xx_cur_int_mask   = ICP201XX_INT_MASK_FIFO_WMK_HIGH | ICP201XX_INT_MASK_FIFO_WMK_LOW ;
			inv_run_icp201xx_in_interrupt_mode(icp201xx_default_op_mode,0);
		}
		else if ( ICP201XX_RUN_MODE_POLLING == icp201xx_default_run_mode )
			inv_run_icp201xx_in_polling(icp201xx_default_op_mode,0);
		
	}
	
	while(1)
	{
		delay_us(1000);
	}

}

/*
 * Printer function for message facility
 */
 void inv_msg(int level, const char * str, ...)
 {
	 /* TO_DO : Implement logging function*/
 }



