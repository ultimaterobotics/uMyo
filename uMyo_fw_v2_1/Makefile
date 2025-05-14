PROJECT_NAME     := uMyo
TARGETS          := uMyo
OUTPUT_DIRECTORY := build/

SDK_ROOT := urf_lib/nrf_usdk52
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/uMyo.out: \
  LINKER_SCRIPT  := ../urf_lib/ur_device_nrf52810.ld 

# Source files common to all targets
SRC_FILES += \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/leds.c \
  $(PROJ_DIR)/adc_read.c \
  $(PROJ_DIR)/fft_opt.c \
  $(PROJ_DIR)/spim_functions.c \
  $(PROJ_DIR)/lsm6ds3.c \
  $(PROJ_DIR)/quat_math.c \
  $(PROJ_DIR)/persistent_storage.c \
  $(PROJ_DIR)/../urf_lib/urf_radio.c \
  $(PROJ_DIR)/../urf_lib/urf_timer.c \
  $(PROJ_DIR)/../urf_lib/urf_star_protocol.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_peripheral.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_encryption.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_smp_process.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_att_process.c \
  $(SDK_ROOT)/gcc/gcc_startup_nrf52810.S \
  $(SDK_ROOT)/system_nrf52810.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(PROJ_DIR)/../urf_lib \
  $(SDK_ROOT)/device \
  $(SDK_ROOT)/cmsis/include \
  $(SDK_ROOT) \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF52
CFLAGS += -DNRF52810_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall
CFLAGS += -mfloat-abi=soft
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums 

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=soft
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DNRF52810_XXAA
ASMFLAGS += -DNRF52_PAN_74

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default

# Default target - first one defined
default: uMyo

TEMPLATE_PATH := $(SDK_ROOT)/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

