ifeq ($(obj),)
obj=	.
endif

TOP = $(obj)/..

obj-m		+= ath_hal.o
ath_hal-objs	:= ah_osdep.o ath5k_hw.o ieee80211_regdomain.o

include $(TOP)/Makefile.inc

INCS += -I$(TOP) -I$(HAL)

EXTRA_CFLAGS+= $(INCS) $(COPTS)

-include $(TOPDIR)/Rules.make

all:	ah_osdep.c ah.h ieee80211_regdomain.h
	$(MAKE) -C $(KERNELPATH) SUBDIRS=$(shell pwd) modules

install:
	test -d $(DESTDIR)/$(KMODPATH) || mkdir -p $(DESTDIR)/$(KMODPATH)
	cp ath_hal.$(KMODSUF) $(DESTDIR)/$(KMODPATH)

clean:
	-rm -f *~ *.o *.ko *.mod.c
	-rm -f .depend .version .*.o.flags .*.o.d .*.o.cmd .*.ko.cmd
	-rm -rf .tmp_versions

ath_hal.o: $(ath_hal-objs)
	$(LD) $(LDOPTS) -o ath_hal.$(KMODSUF) -r $(ath_hal-objs)
