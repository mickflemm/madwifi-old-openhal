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

#
# Makefile for the HAL-based Atheros driver.
#
DEPTH=	.

include Makefile.inc

# NB: the order is important here
DIRS=	$(ATH_HAL) $(ATH_RATE) $(WLAN) $(ATH)

all: configcheck
	mkdir -p $(SYMBOLSDIR)
	for i in $(DIRS); do \
		(cd $$i; make) || exit 1; \
	done

install:
	for i in $(DIRS); do \
		(cd $$i; make install) || exit 1; \
	done
	@if [ -z $(DESTDIR) ]; then \
	    /sbin/depmod -ae ; \
	elif [ -f $(SYSTEMMAP) ]; then \
	    /sbin/depmod -ae -b $(DESTDIR) -F $(SYSTEMMAP) $(KERNELRELEASE) ; \
	else \
	    echo "Don't forget to run \"depmod -ae\" on the target system."; \
	fi

FILES=	COPYRIGHT ath include Makefile Makefile.inc \
	README release.h share net80211 ath_rate
HAL_TAG=ATH_HAL_20030802

release:
	DATE=`date +'%Y%m%d'`; TAG="MADWIFI_$$DATE"; DIR="madwifi-$$DATE"; \
	cvs tag -F $$TAG $(FILES); \
	rm -rf $$DIR; mkdir $$DIR; \
	cvs export -d $$DIR -r $$TAG linux; \
	(cd $$DIR; cvs export -r $(HAL_TAG) hal); \
	tar zcf $$DIR.tgz --exclude=CVS --exclude=hal/freebsd $$DIR; \
	rm -rf $$DIR

clean:
	for i in $(DIRS); do \
		(cd $$i; make clean); \
	done
	rm -rf $(SYMBOLSDIR)

info:
	@echo "The following settings will be used for compilation:"
	@echo "OS           : $(OS)"
	@echo "BUS          : $(BUS)"
	@if [ ! -z "$(TOOLPATH)" ]; then \
	    @echo "TOOLPATH     : $(TOOLPATH)"; \
	fi	
	@echo "KERNELRELEASE: $(KERNELRELEASE)"
	@echo "KERNELPATH   : $(KERNELPATH)"
	@echo "KERNELCONF   : $(KERNELCONF)"
	@echo "MODULEPATH   : $(MODULEPATH)"
	@echo "KMODSUF      : $(KMODSUF)"
	@if [ ! -z "$(DESTDIR)" ]; then \
	    echo "DESTDIR      : $(DESTDIR)"; \
	    echo "SYSTEMMAP    : $(SYSTEMMAP)"; \
	fi

dotconfig:
	@#
	@# check if kernel configuration file is available
	@#	
	@if [ ! -f $(KERNELCONF) ]; then \
	    echo "You need to configure your kernel."; \
	    exit 1; \
	fi

# new targets should be inserted ABOVE this line in order to avoid
# problems with the included kernel configuration file below.
include $(KERNELCONF)

configcheck: dotconfig
	@echo -n "Checking if all requirements are met... "
	
	@# check version of kernel and wireless.h
	@echo $(KERNELRELEASE) | grep -q -i '^[2-9]\.[4-9]\.' || { \
	    echo "FAILED"; \
	    echo "Only kernel versions 2.4.x and above are supported."; \
	    echo "You have $(KERNELRELEASE)."; \
	    exit 1; \
	}
	
	@# check kernel configuration
	@if [ -z "$(CONFIG_SYSCTL)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable sysctl support."; \
	    exit 1; \
	fi
	@if [ -z "$(CONFIG_NET_RADIO)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable wireless extensions."; \
	    exit 1; \
	fi
	@if [ -z "$(CONFIG_CRYPTO)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable crypto API."; \
	    exit 1; \
	fi
	
	@# some other sanity checks
	@if [ ! -d $(ATH_RATE) ]; then \
	    echo "FAILED"; \
	    echo "Selected rate control $(ATH_RATE) not available."; \
	    exit 1; \
	fi
		
	@echo "ok."
