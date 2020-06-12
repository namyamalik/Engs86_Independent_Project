#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Applications/ti/simplelink_cc2640r2_sdk_4_10_00_10/source;/Applications/ti/simplelink_cc2640r2_sdk_4_10_00_10/kernel/tirtos/packages
override XDCROOT = /Applications/ti/xdctools_3_51_03_28_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Applications/ti/simplelink_cc2640r2_sdk_4_10_00_10/source;/Applications/ti/simplelink_cc2640r2_sdk_4_10_00_10/kernel/tirtos/packages;/Applications/ti/xdctools_3_51_03_28_core/packages;..
HOSTOS = MacOS
endif
