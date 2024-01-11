/*******************************************************************************
* Copyright (C) 2023 Maxim Integrated Products, Inc. (now owned by Analog
* Devices, Inc.), Analog Devices, Inc. All Rights Reserved. This software
* is proprietary and confidential to Analog Devices, Inc. and its licensors.
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
*/

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sfcc.h"
#include "sfcc_reva.h"

int MXC_SFCC_ID(mxc_sfcc_info_t cid)
{
    return MXC_SFCC_RevA_ID ((mxc_sfcc_reva_regs_t*) MXC_SFCC, cid);
}

void MXC_SFCC_Enable(void)
{
    MXC_SFCC_RevA_Enable ((mxc_sfcc_reva_regs_t*) MXC_SFCC);
}

void MXC_SFCC_Disable(void)
{

    MXC_SFCC_RevA_Disable ((mxc_sfcc_reva_regs_t*) MXC_SFCC);
}

void MXC_SFCC_Flush(void)
{    
    /* NOTE: MEMPROT authentication bytes are not flushed with the SFCC invalidate bits,
     * the GCR SFCC flush is required.
     */
    /* Flush all instruction caches */
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_SFCC_FLUSH;
    
    /* Wait for flush to complete */
    while (MXC_GCR->sysctrl & MXC_F_GCR_SYSCTRL_SFCC_FLUSH) {
    }

    MXC_SFCC_Disable();
    MXC_SFCC_Enable();
}
