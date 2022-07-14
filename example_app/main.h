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
#ifndef _MAIN_H_
#define _MAIN_H_
#ifdef __cplusplus
extern "C" {
	#endif
#define ICP201XX_RUN_MODE_INTERRUPT_FIFO_OVERFLOW           1 /** FIFO overflow/underflow interrupt mode **/
#define ICP201XX_RUN_MODE_INTERRUPT_FIFO_WATERMARK          2 /** FIFO watermark high/low interrupt mode **/
#define ICP201XX_RUN_MODE_POLLING                           3 /** Polling mode  **/
#define ICP201XX_RUN_MODE_MEAS_ON_USER_REQUEST              4 /** Get SET OF measurements from ICP201xx only when SW0 button is pressed else stay in stand by */

#define POLLING_MODE_FIFO_THRESOLD                         12  /** Min number of packets in FIFO to issue read while doing polling**/
#define FIFO_WATERMARK_HIGH_VALUE                          0X0A
#define FIFO_WATERMARK_LOW_VALUE                           1

/* Pressure Abs and Pressure Delta interrupt tests are only shown with ICP201XX_RUN_MODE_POLLING mode enabled for simplicity in this application **/
#define EN_PRESS_ABS_INT_TEST           0    /** default run mode should be set to  ICP201XX_RUN_MODE_POLLING before enabling testing PRESS ABS Int test ****/
#define EN_PRESS_DELTA_INT_TEST         0    /** default run mode should be set to  ICP201XX_RUN_MODE_POLLING before enabling testing PRESS Delta Int test ****/

#define PRESS_DELTA_VALUE               102   /** PRESS_DELTA = (Pdelta/80)* 2^14 ; 0.5 kpa = 102  **/
#define PRESS_ABS_VALUE                 6759  /** PABS = (P(kPa)-70kPa)/40kPa*2^13 103 Kpa = 6759 **/


/* Function Prototype Declaration */

void msg_printer(int level, const char * str, va_list ap);

/** @brief Config ICP201xx in polling mode and reads measurements for requested number of sec. 
 * @param[in] op_mode op mode to configure ICP201xx.
 * @param[in] run_time_in_sec Expected number of seconds polling mode should run. 
 *            0 if polling mode is expected to run indefinitely 
 */
void inv_run_icp201xx_in_polling(icp201xx_op_mode_t op_mode,uint32_t run_time_in_sec);

/** @brief Config ICP201xx in Interrupt mode and reads measurements for requested number of sec. 
 * @param[in] op_mode op mode to configure ICP201xx.
 * @param[in] run_time_in_sec Expected number of seconds Interrupt mode should run. 
 *            0 if Interrupt mode is expected to run indefinitely 
 *     
 */                
void inv_run_icp201xx_in_interrupt_mode(icp201xx_op_mode_t op_mode,uint32_t run_time_in_sec);

/** @brief Config ICP201xx in Trigger mode and keeps triggering ICP201xx for measurements .
 * @param[in] run_time_in_sec Expected number of seconds trigger mode should run.
 *            0 if trigger mode is expected to run indefinitely
 */ 
void inv_run_icp201xx_in_trigger_mode(uint32_t run_time_in_sec);

#ifdef __cplusplus
}
	#endif
#endif /* !_MAIN_H_ */
