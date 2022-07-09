/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*
*	@file		hsm.c
*	@author		Benjamin VINOT - <benjamin.vinot@maximintegrated.com>
*   @date		Jul 31, 2018
*
*/

#include <ca_sign_build.h>
#include <maxim_c_utils.h>



int hsm_login(void){


#ifdef _MAXIM_HSM
	int i;
	int resu;


	print_info ("HSM try open connection\n");
	/* Open HSM Session and Login using OCS Cards */
	resu = CKR_GENERAL_ERROR;
	for(i = 0; (i<3 && resu != CKR_OK); i++)
	{
		resu = HSM_Login (&session, config_g.hsm_slot_nb);
	}
	if (ERR_OK != resu)
	{
		print_error ("Unable to open connection with HSM.\n");
		HSM_pError (resu);
		return resu;
	}
#endif /* _MAXIM_HSM */



	return ERR_OK;
}


int hsm_close(void){


#ifdef _MAXIM_HSM
	int resu;

	/* Close HSM Sessions */
		resu = HSM_Close (session);
		if (ERR_OK != resu)
		{
			print_warn ("Warning: fails to close HSM connection\n");
		}
#endif /* _MAXIM_HSM */



	return ERR_OK;
}


