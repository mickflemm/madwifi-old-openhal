/*-
 * Copyright (c) 2007 Pavel Roskin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef _ATH_HW_H
#define _ATH_HW_H

#define ATH_HW_IRQ_PENDING		0x4008
# define ATH_HW_IRQ_PENDING_FALSE	0
# define ATH_HW_IRQ_PENDING_TRUE	1

struct ath_hal;

/*
 * Read from a device register
 */
static inline u32 ath_hw_reg_read(struct ath_hal *hw, u16 reg)
{
	return readl(hw->ah_sh + reg);
}

/*
 * Write to a device register
 */
static inline void ath_hw_reg_write(struct ath_hal *hw, u32 val, u16 reg)
{
	writel(val, hw->ah_sh + reg);
}

/*
 * Check if there is an interrupt waiting to be processed.
 * Return 1 if there is an interrupt for us, or 0 if there is none or if
 * the device has been removed.
 */
static inline int ath_hw_irq_pending(struct ath_hal *hw)
{
	if (ath_hw_reg_read(hw, ATH_HW_IRQ_PENDING) == ATH_HW_IRQ_PENDING_TRUE)
		return 1;
	else
		return 0;
}

#endif				/* _ATH_HW_H */
