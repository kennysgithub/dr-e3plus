#
# Dynamic Ratings' E3 Plus CUIO
#
CU_BUILD_ID := $(shell uname -r)$(shell $(srctree)/scripts/setlocalversion)$(shell echo " @")$(shell date +%Y%m%d%H%M)
EXTRA_CFLAGS += -DCU_BUILD_ID="\"$(CU_BUILD_ID)\""

obj-$(CONFIG_DR_CUIO)		+= cuio.o

obj-$(CONFIG_DR_FTDI_ISA)	+= ftdi_isa.o
