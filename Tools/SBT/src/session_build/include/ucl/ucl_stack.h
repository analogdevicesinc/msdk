/*============================================================================
 *
 * ucl_stack.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary informatDATAn of
 * Innova Card ("Confidential InformatDATAn"). You shall not
 * disclose such Confidential InformatDATAn and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Innova Card.
 *
 * Innova Card makes no representatDATAns or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose : UCL Stack
 *
 *==========================================================================*/
#ifndef _UCL_STACK_H_
#define _UCL_STACK_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_stack.h
 * @defgroup UCL_STACK UCL Stack
 * UCL Stack Management.
 *
 * @par Header:
 * @link ucl_stack.h ucl_stack.h @endlink
 *
 * To optimize the temporary data size used by cryptolib, a special data zone
 * called UCL Stack (CS) with management functions has been implemented.@n
 * The CS size is fixed (it is a global variable in section @p .data)
 * at the compilation and the management of the memory is exactly as the
 * system stack. @n
 * @n
 * The CS allows to allocate temporary memory in the critical applications like
 * RSA. It is a simply dynamic allocation with implementation according to
 * the embedded system constraints. @n
 * @n
 * It is possible to use another memory space instead of the default stack. @n
 *
 *
 *
 * @ingroup UCL_DATA
 */


/*============================================================================*/
/** <b>UCL Stack Initialisation</b>.
 * Reserve memory for stack.
 *
 * This function initialize the stack. It is possible to use an another
 * memory space than the default. If the preconditions are not verified the
 * default UCL stack is used.
 *
 * @pre @p pt_stack is not #NULL and @p size is greater than 1024
 * @pre @p size must be a multiple of 4
 *
 * @param[in] pt_stack Pointer to a memory space
 * @param[in] size     32-bit word size
 *
 * @return Error code
 *
 * @retval #UCL_STACK_DEFAULT use default UCL stack
 * @retval #UCL_OK            use an another mermory space for UCL stack
 *
 * @ingroup UCL_STACK
 */
int ucl_stack_init(u32 *pt_stack, int size);


/*============================================================================*/
/** <b>UCL Stack Allocation</b>.
 *
 * @param[out] pt     Pointer to the UCL stack pointer
 * @param[in]  nbWord UCL stack allocation size
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_STACK_NOT_INIT UCL stack is not initialized
 * @retval #UCL_STACK_OVERFLOW Size is greater than memory available
 *
 * @ingroup UCL_STACK
 */
int ucl_stack_alloc(u32 **pt, int nbWord);


/*============================================================================*/
/** <b>Free UCL Stack Allocation</b>.
 *
 * @param[in,out] pt Pointer to UCL stack pointer
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_STACK_NOT_INIT if UCL stack is not initialized
 *
 * @ingroup UCL_STACK
 */
int ucl_stack_free(u32 **pt);


/*============================================================================*/
/** <b>UCL Stack Size</b>.
 *
 * @return UCL Stack size or Error code
 *
 * @retval #UCL_STACK_NOT_INIT if UCL stack is not initialized
 *
 * @ingroup UCL_STACK
 */
int ucl_stack_size(void);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_STACK_H_ */
