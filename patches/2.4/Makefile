O_TARGET := madwifi.o

subdir-$(CONFIG_ATHEROS)		+= ath net80211 openhal
subdir-$(CONFIG_ATHEROS_RATE_SAMPLE)	+= ath_rate/sample
subdir-$(CONFIG_ATHEROS_RATE_ONOE)	+= ath_rate/onoe
subdir-$(CONFIG_ATHEROS_RATE_AMRR)	+= ath_rate/amrr

ifeq ($(CONFIG_ATHEROS),y)
ifeq ($(strip $(BUS)),AHB)
	obj-y += ath/ath_ahb.o
else
	obj-y += ath/ath_pci.o
endif
	obj-y += openhal/ath_hal.o
	obj-$(CONFIG_ATHEROS_RATE_SAMPLE) += ath_rate/sample/ath_rate.o
	obj-$(CONFIG_ATHEROS_RATE_ONOE) += ath_rate/onoe/ath_rate.o
	obj-$(CONFIG_ATHEROS_RATE_AMRR) += ath_rate/amrr/ath_rate.o
	obj-y += net80211/net80211.o
endif

include $(TOPDIR)/Rules.make
