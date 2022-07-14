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

#ifndef _INV_MESSAGE_H_
#define _INV_MESSAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @brief message level definition
 */
enum inv_msg_level {
	INV_MSG_LEVEL_OFF     = 0,
	INV_MSG_LEVEL_ERROR,
	INV_MSG_LEVEL_WARNING,
	INV_MSG_LEVEL_INFO,
	INV_MSG_LEVEL_VERBOSE,
	INV_MSG_LEVEL_DEBUG,
	INV_MSG_LEVEL_MAX
};

/** @brief Display a message (through means of printer function)
 *  @param[in] 	level for the message
 *  @param[in] 	str   message string
 *  @param[in] 	...   optional arguments
 *  @return none
 */
void inv_msg(int level, const char * str, ...);


#define _INV_MSG(level, ...)           inv_msg(level, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* INV_MESSAGE_H_ */

/** @} */
