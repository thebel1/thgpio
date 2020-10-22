# Copyright 2014-2016,2018,2020 VMware, Inc.  All rights reserved.
# -- VMware Confidential

#**********************************************************************---------
# Makefile
#
#    Provides a wrapper to compile drivers in the VMKAPI DDK devkit.
#
#    This is a generic Makefile and no changes are required to compile
#    drivers provided that a single .sc file exists in the current directory.
#**********************************************************************---------

#**********************************************************************---------
# Tools
#**********************************************************************---------

CAYMAN_ESX_GDB_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_esx_gdb-1af75dbb095cc2348c35c6708c8b326ac1661974-toolchain)
export GOBUILD_CAYMAN_ESX_GDB_ROOT = $(CAYMAN_ESX_GDB_DIR)
CAYMAN_ESX_TOOLCHAIN_GCC6_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_esx_toolchain_gcc6-978272870a6446e021754e84f5d75e0c6c53a9b7-toolchain)
export GOBUILD_CAYMAN_ESX_TOOLCHAIN_GCC6_ROOT = $(CAYMAN_ESX_TOOLCHAIN_GCC6_DIR)
CAYMAN_ESX_PYCPARSER_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_esx_pycparser-2f71deb8b1193677d4c68926ba0e85b65e6adc10-toolchain)
export GOBUILD_CAYMAN_ESX_PYCPARSER_ROOT = $(CAYMAN_ESX_PYCPARSER_DIR)
CAYMAN_ESX_NCURSES_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_esx_ncurses-9bf45330b538505c6325906c08c4ecf834184b5c-toolchain)
export GOBUILD_CAYMAN_ESX_NCURSES_ROOT = $(CAYMAN_ESX_NCURSES_DIR)
CAYMAN_LLVM_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_llvm-e51314a6e421afc0fc8856057af8b195bb9805f3-toolchain)
export GOBUILD_CAYMAN_LLVM_ROOT = $(CAYMAN_LLVM_DIR)
CAYMAN_ESX_GLIBC_2_17_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_esx_glibc_2_17-8ef7e61952433909958dd15ee027ca4e03896da7-toolchain)
export GOBUILD_CAYMAN_ESX_GLIBC_2_17_ROOT = $(CAYMAN_ESX_GLIBC_2_17_DIR)
CAYMAN_PYTHON_DIR = $(shell /bin/rpm -q --qf "%{instprefixes}\n" vmware-esx-cayman_python-99788ba0a9e40ea80ae7b05efb941377abfb31e4-toolchain)
export GOBUILD_CAYMAN_PYTHON_ROOT = $(CAYMAN_PYTHON_DIR)
VMW_COMPONENTS = cayman_esx_gdb cayman_esx_toolchain_gcc6 cayman_esx_pycparser cayman_esx_ncurses cayman_llvm cayman_esx_glibc_2_17 cayman_python
VMW_COMPONENT_DIRS = $(CAYMAN_ESX_GDB_DIR) $(CAYMAN_ESX_TOOLCHAIN_GCC6_DIR) $(CAYMAN_ESX_PYCPARSER_DIR) $(CAYMAN_ESX_NCURSES_DIR) $(CAYMAN_LLVM_DIR) $(CAYMAN_ESX_GLIBC_2_17_DIR) $(CAYMAN_PYTHON_DIR)

AWK = /build/toolchain/lin64/gawk-3.1.5/bin/awk
BASH = /build/toolchain/lin32/bash-3.2/bin/bash
CP = /build/toolchain/lin64/coreutils-8.6/bin/cp
ECHO = /build/toolchain/lin64/coreutils-8.6/bin/echo
GREP = /build/toolchain/lin64/grep-2.5.1a/bin/grep
GZIP = /build/toolchain/lin64/gzip-1.5/bin/gzip
MAKE = /build/toolchain/lin64/make-3.81/bin/make
MKDIR = /build/toolchain/lin64/coreutils-8.6/bin/mkdir
MV = /build/toolchain/lin64/coreutils-8.6/bin/mv
PERL = /build/toolchain/lin32/perl-5.8.8/bin/perl
RM = /build/toolchain/lin64/coreutils-8.6/bin/rm
SED = /build/toolchain/lin64/sed-4.1.5/bin/sed
TR = /build/toolchain/lin64/coreutils-8.6/bin/tr

#**********************************************************************---------
# Parameters to the scons command
#**********************************************************************---------
PRODUCT      = nativeddkarm64
BUILDTYPE    = release
BUILD_NUMBER = 40650718

VMKAPI_DIR   = $(shell /bin/rpm -q --qf '%{INSTPREFIXES}\n' vmware-esx-nativeddkarm64-devtools-7.0.1-1.0.40650718)
SRC          = $(VMKAPI_DIR)/src
FIND_RPM     = $(VMKAPI_DIR)/tools/find_rpm_install_dir.sh
VIRTUAL_GOBUILD = $(VMKAPI_DIR)/tools/gobuild/nativeddk.cache
EPKDIR      := $(shell $(FIND_RPM) vmware-esx-packaging-kit)
ifeq ($(strip $(EPKDIR)),)
    $(error Unable to locate ESX Packaging Kit installation directory)
endif

SCONS        = cd $(SRC); $(SRC)/scons/bin/scons --config=force
FULL_PWD     := $(shell pwd)
SC           := $(shell echo *.sc | $(AWK) '{print $$1}')
MANIFEST     = $(FULL_PWD)/$(SC)
BUILD        = build
SCONS_BUILD  = .build
TARGET       = $(FULL_PWD)/$(BUILD)
SCONS_TARGET = $(FULL_PWD)/$(SCONS_BUILD)

# Map VIB acceptance level to canonical default signing-key file base name
ACCEPT_LEVEL = $(shell grep acceptance-level $(MANIFEST))
ifneq ($(findstring accepted,$(ACCEPT_LEVEL)),)
    VMW_KC_NAME = accepted
else
ifneq ($(findstring partner,$(ACCEPT_LEVEL)),)
    VMW_KC_NAME = vmpartner
else
ifneq ($(findstring certified,$(ACCEPT_LEVEL)),)
    VMW_KC_NAME = vmware
else
ifneq ($(findstring community,$(ACCEPT_LEVEL)),)
    VMW_KC_NAME = community
else
ifeq ($(findstring community,$(ACCEPT_LEVEL)),)
    $(error invalid acceptance level $(ACCEPT_LEVEL))
endif
endif
endif
endif
endif
ifneq ($(VMW_KC_NAME),)
    KEYPATH?=$(EPKDIR)/testcerts/$(VMW_KC_NAME).key
    CERTPATH?=$(EPKDIR)/testcerts/$(VMW_KC_NAME).cert
endif

all vib:
	@$(SCONS) native-driver-modules native-driver-vibs native-driver-bundle \
	       RELEASE_PACKAGES_DIR=$(TARGET) \
	       TOOLKIT_MANIFEST=$(MANIFEST) \
	       PRODUCT=$(PRODUCT) \
	       BUILDTYPE=$(BUILDTYPE) \
	       BUILD_NUMBER=$(BUILD_NUMBER) \
	       ESX_SIGN_BINARIES=0 \
	       EPKPATH=$(EPKDIR) \
	       BUILDROOT=$(SCONS_TARGET) \
	       VIRTUAL_GOBUILD=$(VIRTUAL_GOBUILD) \
               KEYPATH=$(KEYPATH) \
               CERTPATH=$(CERTPATH)

clean:
	@echo Cleaning out build directories ...
	@$(RM) -rf $(TARGET) $(SCONS_TARGET)
	@echo Done.
