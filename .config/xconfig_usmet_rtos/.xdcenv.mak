#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/brandon/ti/tirtos_tivac_2_16_00_08/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;/home/brandon/workspace_v9/uSMET/.config
override XDCROOT = /home/brandon/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/brandon/ti/tirtos_tivac_2_16_00_08/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;/home/brandon/ti/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;/home/brandon/workspace_v9/uSMET/.config;/home/brandon/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = Linux
endif
