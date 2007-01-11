ifeq ($(obj),)
obj=	.
endif

DEPTH=	..

obj-m		+= ath_hal.o
ath_hal-objs	:= ah_osdep.o ar5xxx.o ar5212.o ar5211.o ar5210.o ieee80211_regdomain.o
export-objs	:= ah_osdep.o
list-multi	:= ath_hal.o

include $(obj)/${DEPTH}/Makefile.inc

INCS+=	-I${HAL} -I${HAL}/${OS} -I${obj}/${ATH_HAL} -I${obj}/${DEPTH} -I${WLAN}

EXTRA_CFLAGS+= ${INCS} ${COPTS}

-include $(TOPDIR)/Rules.make

all:	ah_osdep.c ah.h ieee80211_regdomain.h
	$(MAKE) -C $(KERNELPATH) SUBDIRS=$(shell pwd) MODVERDIR=$(shell pwd)/${SYMBOLSDIR} modules

install:
	test -d ${DESTDIR}/${MODULEPATH} || mkdir -p ${DESTDIR}/${MODULEPATH}
	$(STRIP) -S ath_hal.$(KMODSUF)
	cp ath_hal.${KMODSUF} ${DESTDIR}/${MODULEPATH}

clean:
	-rm -f *~ *.o *.ko *.mod.c
	-rm -f .version .*.o.flags .*.o.d .*.o.cmd .*.ko.cmd
	-rm -rf .tmp_versions

ath_hal.o: ${ath_hal-objs}
	$(LD) $(LDOPTS) -o ath_hal.$(KMODSUF) -r $(ath_hal-objs)
