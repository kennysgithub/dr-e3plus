#
# Dynamic Ratings' E3 Plus CUIO
#
CU_BUILD_ID := $(shell uname -r)$(shell $(srctree)/scripts/setlocalversion)$(shell echo " @")$(shell date +%Y%m%d%H%M)
EXTRA_CFLAGS += -DCU_BUILD_ID="\"$(CU_BUILD_ID)\""

obj-$(CONFIG_DR_CUIO)		+= dr_cuio.o

dr_cuio-objs := cuio.o
