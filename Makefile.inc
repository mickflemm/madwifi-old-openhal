#
# Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce at minimum a disclaimer
#    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
#    redistribution must be conditioned upon including a substantially
#    similar Disclaimer requirement for further binary redistribution.
# 3. Neither the names of the above-listed copyright holders nor the names
#    of any contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# Alternatively, this software may be distributed under the terms of the
# GNU General Public License ("GPL") version 2 as published by the Free
# Software Foundation.
#
# NO WARRANTY
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGES.
#
# $Id$
#

ifeq ($(obj),)
obj=	.
endif

# other locales sometimes cause trouble
export LC_ALL = POSIX

# OS is the target operating system. Currently only Linux is supported.
OS=		linux

# TARGET defines the target platform ISA per GNU rules.
# It must match one of the target platforms supported by
# the HAL.  To see the set of target platforms look at
# hal/linux/*.inc.  You can set this in the environment
# to override the default setting.
#
ifeq ($(TARGET),)
TARGET=		i386-elf
endif

# BUS defines the bus type to which the wireless devices is attached.
# Currently, the valid BUS types are PCI and AHB.  If BUS is not
# defined, then, we assume the bus type is PCI
ifndef BUS
BUS=		PCI
endif

# DESTDIR is used as path prefix during installation.
DESTDIR ?=

# KERNELPATH is the path to the target kernel's build/source area.
# This is used to obtain the kernel configuration and include files.
# If automatic determination doesn't work as desired, you have to
# manually set it, for example with:
# make KERNELPATH=/path/to/kernel/source

# TOOLPATH is the path which contains the crosscompile toolchain (?)

ifeq ($(strip ${BUS}),AHB)
# Bus type AHB forces the target platform to be mipsisa32.
TARGET :=	mipsisa32-be-elf

# Bus type AHB requires KERNELPATH and TOOLPATH to be set manually.
ifndef KERNELPATH
$(error KERNELPATH must be defined for bus type AHB.)
endif
ifndef TOOLPATH
$(error TOOLPATH must be defined for bus type AHB.)
endif

else

KERNELPATH ?= $(firstword $(wildcard $(DESTDIR)/lib/modules/$(shell uname -r)/build /usr/src/linux))

ifeq ($(KERNELPATH),)
$(error KERNELPATH must be defined)
endif

endif

# sanity check: does KERNELPATH exist?
ifeq ($(wildcard $(KERNELPATH)),)
$(error KERNELPATH: $(KERNELPATH) does not exist)
endif

# KERNELRELEASE is the target kernel's version.  If KERNELRELEASE
# is not set in the environment then it is taken from the running
# system.
# To determine the correct version a mini-Makefile is piped to make
# which includes the Makefile in KERNELPATH and prints out the version.
# This is done to come by problems recently introduced with some distros
# and newer kernel releases.
#
# Note: '\044' used instead of '$' in order to avoid make complaining like
# "Recursive variable `KERNELRELEASE' references itself (eventually)."
KERNELRELEASE ?= $(shell echo -e 'foo:\n\t@echo \044(KERNELRELEASE)\ninclude Makefile' | make -C $(KERNELPATH) -s -f-)

# KERNELCONF is the name of the file that holds the configuration
# of the target kernel.
KERNELCONF ?=	${KERNELPATH}/.config

# sanity check: does KERNELCONF exist?
ifeq ($(wildcard $(KERNELCONF)),)
$(error KERNELCONF: $(KERNELCONF) does not exist.)
endif

# SYSTEMMAP is the name of the System.map file for the target
# kernel. This will be needed when DESTDIR is defined in order
# to run depmod properly.
SYSTEMMAP ?= 	${KERNELPATH}/System.map

# MODULEPATH nominates the directory where the modules will be
# installed to
ifeq ($(strip ${BUS}),AHB)
MODULEPATH := 	${KERNELPATH}/arch/mips/ar531x/RAMDISK/rootdir/lib/modules/${KERNELRELEASE}/net
else
MODULEPATH ?=	/lib/modules/${KERNELRELEASE}/net
endif

# WIRELESSEXT contains information about the version of the wireless
# extensions that are available in the target kernel
WIRELESSEXT ?=	$(shell cat $(KERNELPATH)/include/linux/wireless.h | grep "\#define WIRELESS_EXT" | cut -f2)

# Some settings that depend on actual kernel release
ifneq ($(findstring 2.6,$(KERNELRELEASE)),)
export-objs	:=
list-multi	:=
KMODSUF		:= ko
else
KMODSUF		:= o
endif

NM=            nm
AWK=           awk

#
# Path to the HAL source code.
#
ifeq ($(HAL),)
HAL=	${obj}/${DEPTH}/hal
endif
#
# Path to the HAL build area.
#
ATH_HAL=${DEPTH}/ath_hal
#
# Path to the 802.11 include files.
#
WLAN=	${DEPTH}/net80211
#
# Path to the Atheros device driver.
#
ATH=	${DEPTH}/ath
#
# Path to the rate control algorithm.
#
#ATH_RATE=$(shell find ath_rate/ -maxdepth 1 ! -name CVS ! -name ath_rate/ -type d)
ifeq ($(ATH_RATE),)
ATH_RATE=ath_rate/sample
endif

INCS=	-include ${obj}/${DEPTH}/include/compat.h -I${obj}/${DEPTH}/include

ifeq ($(strip ${BUS}),AHB)
INCS+= -isystem ${TOOLPATH}/include
endif

include ${HAL}/public/${TARGET}.inc

SYMBOLSDIR=${DEPTH}/symbols
