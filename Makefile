# This file is generated for compiling freeRTOS as a library
###############################################################################

######################################
# target
######################################
TARGET = $(BUILD_DIR)/libfreeRTOS

export MAINBASE := $(shell pwd)
SRCBASE := $(MAINBASE)/FreeRTOS/Source

GCC_BIN := ~/opt/gcc-arm-none-eabi-6-2017-q2-update/bin/

#######################################
# Build path
BUILD_DIR = build

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og



######################################
# source
######################################
# C sources
C_SOURCES =  \
$(SRCBASE)/list.c \
$(SRCBASE)/portable/GCC/ARM_CM4F/port.c \
$(SRCBASE)/croutine.c \
$(SRCBASE)/queue.c \
$(SRCBASE)/timers.c \
$(SRCBASE)/portable/MemMang/heap_4.c \
$(SRCBASE)/tasks.c \
$(SRCBASE)/CMSIS_RTOS/cmsis_os.c \
$(SRCBASE)/event_groups.c \


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
CC 		= $(GCC_BIN)arm-none-eabi-gcc
AS 		= $(GCC_BIN)arm-none-eabi-gcc -x assembler-with-cpp
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
LD 		= $(GCC_BIN)arm-none-eabi-gcc
AR 		= $(GCC_BIN)arm-none-eabi-ar
SIZE	= $(GCC_BIN)arm-none-eabi-size
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
PREPRO	= $(GCC_BIN)arm-none-eabi-cpp
RANLIB	= $(GCC_BIN)arm-none-eabi-ranlib

HEX = $(OBJCOPY) -O ihex
BIN = $(OBJCOPY) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32L476xx

# AS includes
AS_INCLUDES =  \
-IInc


# C includes
C_INCLUDES =  \
-IInc \
-I$(SRCBASE)/portable/GCC/ARM_CM4F \
-I$(SRCBASE)/include \
-I$(SRCBASE)/CMSIS_RTOS \


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
CFLAGS += -DSTM32L476xx=1
CFLAGS += -DTARGET_NUCLEO_L476RG
CFLAGS += -DCPU_STM32L476RG

AR_FLAGS = -rcs

# default action: build all
all: $(TARGET).a size

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(TARGET).a: $(OBJECTS)
	$(AR) -rcs $@ $^

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

size: $(TARGET).a
	$(SIZE) $(TARGET).a

$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***