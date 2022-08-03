################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager 
# ('make modlibs' from command line), which will also update Eclipse IDE launch 
# configurations. If TARGET is manually edited, ensure TARGET_<BSP>.mtb with a 
# valid URL exists in the application, run 'make getlibs' to fetch BSP contents
# and update or regenerate launch configurations for your IDE.
#
TARGET=WLC1115-68LQXQ

# Name of application (used to derive name of final linked file).
# 
# If APPNAME is edited, ensure to update or regenerate launch 
# configurations for your IDE.
APPNAME=mtb-example-wlc1_ptx_epp

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC 7.2.1, provided with ModusToolbox IDE
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
# 
# If CONFIG is manually edited, ensure to update or regenerate launch configurations 
# for your IDE.
CONFIG=Release

# If set to "true" or "1", display full command-lines when building.
VERBOSE=


################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS+=CUSTOM_DESIGN_MODUS WLC1_PD3_SNK WLC1_HPI wlc1_qi_ptx

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS+=BSP_DESIGN_MODUS

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=

# Add additional defines to the build process (without a leading -D).
# Enabled PD revision 3.0 support, VBus OV Fault Protection and Deep Sleep mode in idle states.
#TODO: CY_DEVICE_WLC1 is added for now here, once MPN is supported, it should come from design.modus
DEFINES=CY_DEVICE_WLC1=1 CY_PD_SINK_ONLY=1 CY_PD_SOURCE_ONLY=0 CY_PD_REV3_ENABLE=1 CCG_PD_REV3_ENABLE=1 VBUS_OVP_ENABLE=0 VBUS_UVP_ENABLE=0 SYS_DEEPSLEEP_ENABLE=1 VBUS_CTRL_TYPE_P1=4 CY_PD_VBUS_CF_EN=0 CY_PD_CRC_ERR_HANDLING_ENABLE=1 \
        VBUS_IN_DISCHARGE_EN=1 CY_PD_PPS_SRC_ENABLE=0 VBUS_SCP_ENABLE=0 VBUS_OCP_ENABLE=0 DC_DC=0 VBTR_ENABLE=1 CCG_CF_HW_DET_ENABLE=1 CY_PD_CBL_DISC_DISABLE=1 VBUS_SLOW_DISCHARGE_EN=1 VBUS_CTRL_TRIM_ADJUST_ENABLE=1 \
		CCG_TEMP_BASED_VOLTAGE_THROTTLING=0 USE_CYACD2_METADATA_FORMAT=1 IBTR_ENABLE=1 CCG_PD_BLOCK_ALWAYS_ON=1 BB_MODE_PSM_ONLY=0 CCG_VIN_BASED_VOLTAGE_THROTTLING=0 BB_PWM_ASYNC_MODE_ENABLE=1 CCG_LOAD_SHARING_ENABLE=0 \
        BATTERY_CHARGING_ENABLE=0 LEGACY_PD_PARALLEL_OPER=1 QC_AFC_CHARGING_DISABLED=0 ENABLE_APPLE_BC12_SUPPORT=0 \
		QC_SRC_AFC_CHARGING_DISABLED=1 QC_PPS_ENABLE=0 QC_PPS_CHARGER_TYPE=5 QC_PPS_VERSION=49 \
        NO_OF_BC_PORTS=1 QC2_MAX_CURRENT=CY_PD_I_3A \
        QC_AFC_SNK_EN=1 \
        BC_QC_AFC_SNK_MAX_VOLT=CY_PD_VSAFE_15V BC_QC_AFC_SNK_MIN_VOLT=CY_PD_VSAFE_5V \
        BC_QC_AFC_SNK_MAX_CUR=CY_PD_I_3A BC_QC_AFC_SNK_MIN_CUR=CY_PD_I_1P5A \
        VIN_UVP_ENABLE=0 VIN_OVP_ENABLE=0 VBAT_GND_SCP_ENABLE=0 BB_ILIM_DET_ENABLE=0 VREG_BROWN_OUT_DET_ENABLE=0 VREG_INRUSH_DET_ENABLE=0 DISABLE_CC_OVP_OVER_SLEEP=0 \
        CY_QI_BMC_RX_SPI_INDEX=SCB2 CY_QI_BMC_RX_SPI_PORT_INDEX=2 \
		CY_USE_CONFIG_TABLE=1 \
        CY_SOLN_VBTR_EN=1

#PDL fault requirements
DEFINES+=APP_VBUS_SRC_FET_BYPASS_EN=1 \
        PDL_VBUS_UVP_ENABLE=1 \
        PDL_VBUS_OVP_ENABLE=1 \
        PDL_VBUS_OCP_ENABLE=1 \
        PDL_VBUS_SCP_ENABLE=1 \
        PDL_VIN_UVP_ENABLE=1 \
        PDL_VIN_OVP_ENABLE=1 \
        UVP_OVP_RECOVERY=1\
        PDL_BB_ILIM_DET_ENABLE=1\
        PDL_VREG_BROWN_OUT_DET_ENABLE=1\
        PDL_VREG_INRUSH_DET_ENABLE=1

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
# CFLAGS=-O3 -Os -flto
CFLAGS=-O3 -Os -flto

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
ifeq ($(TOOLCHAIN), GCC_ARM)
LINKER_SCRIPT=wlc1_app.ld
endif

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.

#Post build to merge bootloader and application
ifeq ($(TOOLCHAIN), GCC_ARM)
POSTBUILD+=\
            $(CY_MCUELFTOOL) --sign $(CY_CONFIG_DIR)/$(APPNAME).elf CRC --output $(CY_CONFIG_DIR)/$(APPNAME)_crc.elf && \
            $(CY_MCUELFTOOL) -P $(CY_CONFIG_DIR)/$(APPNAME)_crc.elf --output $(CY_CONFIG_DIR)/$(APPNAME)_crc.cyacd2 && \
            $(CY_MCUELFTOOL) -M bootloader/mtb-example-wicg1-bootloader.elf $(CY_CONFIG_DIR)/$(APPNAME)_crc.elf --output $(CY_CONFIG_DIR)/$(APPNAME)_merged.elf --hex  $(CY_CONFIG_DIR)/$(APPNAME)_merged.hex
else
POSTBUILD+=
endif

#Post build to generate PSoC Creator HEX file from MTB HEX file
ifeq ($(OS),Windows_NT)
# Generate PSoC Programmer Compatible HEX file - $(APPNAME)_pp.hex for WICG1
CY_SILICON_ID=0x318011C1 # WICG1
CY_HEX_FILE=$(CY_CONFIG_DIR)/$(APPNAME)_merged.hex
CY_BIN_FILE=$(CY_CONFIG_DIR)/$(APPNAME)_merged.bin
CY_PP_HEX_FILE=$(CY_CONFIG_DIR)/$(APPNAME)_merged_pp.hex
POSTBUILD+=&&chmod +x utils/hex2bin.exe && ./utils/hex2bin.exe -T 0x20000 -p 0 -m 20000 $(CY_HEX_FILE) &&\
chmod +x utils/bin2psochex.exe && ./utils/bin2psochex.exe $(CY_SILICON_ID) 0 0x100 $(CY_BIN_FILE) $(CY_PP_HEX_FILE)

#Move final binaries to new folder, Final binary will be PSOC based binary; if in need of Modus Generated hex look for it in Release folder
POSTBUILD+=&&mkdir -p $(CY_CONFIG_DIR)/binaries/ && \
            mv $(CY_CONFIG_DIR)/$(APPNAME)_merged_pp.hex $(CY_CONFIG_DIR)/binaries/$(APPNAME).hex && \
            mv $(CY_CONFIG_DIR)/$(APPNAME)_crc.cyacd2 $(CY_CONFIG_DIR)/binaries/$(APPNAME).cyacd2 && \
            mv $(CY_CONFIG_DIR)/$(APPNAME).hex $(CY_CONFIG_DIR)/binaries/$(APPNAME)_MTB.hex && \
            cp $(CY_CONFIG_DIR)/binaries/$(APPNAME).hex $(CY_CONFIG_DIR)/$(APPNAME).hex
endif

# Uncomment this to enable programming the board with MiniProg4 5V power supply
override CY_OPENOCD_ARGS=set ENABLE_POWER_SUPPLY 3300; $(CY_OPENOCD_INTERFACE); $(CY_OPENOCD_TARGET);


################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field 
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by 
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level 
# above the current app directory. This is used with the CY_GETLIBS_SHARED_NAME 
# variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=


# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

# Path to Elf tool directory. 
CY_MCUELFTOOL_DIR=$(wildcard $(CY_TOOLS_DIR)/cymcuelftool-*)

# CY MCU ELF tool executable path.
ifeq ($(OS),Windows_NT)
    CY_MCUELFTOOL=$(CY_MCUELFTOOL_DIR)/bin/cymcuelftool.exe
else
    CY_MCUELFTOOL=$(CY_MCUELFTOOL_DIR)/bin/cymcuelftool
endif


include $(CY_TOOLS_DIR)/make/start.mk
