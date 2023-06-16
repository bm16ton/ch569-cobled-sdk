COMPILER_PREFIX ?= riscv-none-elf


BOARD_PATH = ../boards/ch565w_evk/
include $(BOARD_PATH)/board.mk

# TeenyDT format descriptor
TEENYDT = winusb30_desc.lua
USBMEM = 512
USBEP = 5
# project source
SOURCES  = winusb30.c \
           teeny_usb_desc.c
INCLUDES = .
DEFINES  = __forceinline=inline NO_HOST
# board source
SOURCES  += $(BOARD_SRC)
INCLUDES += $(BOARD_INC)
DEFINES  += $(BOARD_DEF)

# Teeny USB source
TUSB_PATH = ../../core
SOURCES  += $(wildcard $(TUSB_PATH)/*.c)
INCLUDES += $(TUSB_PATH)
INCLUDES += $(TUSB_PATH)/../driver_ch56x
INCLUDES += $(TUSB_PATH)/../driver_ch56x/ch56xlib/RVMSIS

# MCU driver source
SOURCES += $(wildcard $(TUSB_PATH)/../driver_stm32/*.c)

TOOLSET     ?= riscv-none-elf-
CC           = $(TOOLSET)gcc
LD           = $(TOOLSET)gcc
AR           = $(TOOLSET)ar
AS           = $(TOOLSET)as
SIZE         = $(TOOLSET)size
OBJCOPY      = $(TOOLSET)objcopy
RM           = rm -rf
CP           = cp
MKDIR        = mkdir -p

CPU ?= cortex-m3
CCFLAGS     += -march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8 $(OPTIM_OPTS) -fmessage-length=0 -fsigned-char
CCFLAGS     += $(addprefix -D, $(DEFINES))
CCFLAGS     += $(addprefix -I, $(INCLUDES)) std=gnu99 -MMD -MP -MT"$(@)" -DTUSB_SUPPORT_USB3 -DTUSB_SUPPORT_USB2
CCFLAGS     += -ffunction-sections -fdata-sections -specs=nano.specs -specs=nosys.specs
LDFLAGS     += $(CCFLAGS)
LDFLAGS     += -Wl,--script='$(LDSCRIPT)' -Wl,-Map=$(OUTDIR)/$(TARGET).map
LDFLAGS     += -Wl,--gc-sections -T ".ld" -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs


OUTDIR = ./output
OBJDIR = ./obj/$(TARGET)
OBJECTS      = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(notdir $(basename $(SOURCES)))))
DEPENDS      = $(addprefix $(OBJDIR)/, $(addsuffix .d, $(notdir $(basename $(filter %c, $(SOURCES))))))
SRCPATH      = $(sort $(dir $(SOURCES)))
vpath %.c $(SRCPATH)
vpath %.S $(SRCPATH)
vpath %.s $(SRCPATH)

all: ch565w_evk

ch565w_evk:
	@echo Building HydraUSB3_USB
	@$(MAKE) demo TARGET='winusb30' BOARD='ch565w_evk'


demo: $(OUTDIR) $(OUTDIR)/$(TARGET).bin $(OUTDIR)/$(TARGET).hex

$(OBJDIR)/%.o:%.c
	@echo building $<
	@$(CC) $(CCFLAGS) -c $< -o $@
$(OBJDIR)/%.o:%.s
	@echo building $<
	@$(CC) $(ASFLAGS) -c $< -o $@
$(OBJDIR)/%.o:%.S
	@echo building $<
	@$(CC) $(ASFLAGS) -c $< -o $@

$(OBJDIR):
	@$(MKDIR) $@
$(OUTDIR):
	@$(MKDIR) $@

$(OUTDIR)/$(TARGET).hex:$(OUTDIR)/$(TARGET).elf
	@echo generating $@
	@$(OBJCOPY) -Oihex $< $@
$(OUTDIR)/$(TARGET).bin:$(OUTDIR)/$(TARGET).elf
	@echo generating $@
	@$(OBJCOPY) -Obinary $< $@
$(OUTDIR)/$(TARGET).elf: $(OBJDIR) $(OBJECTS)
	@echo linking $@
	@$(LD) $(LDFLAGS) $(OBJECTS) -o $@
	@$(SIZE) $@

USBMEM ?= 512
USBEP  ?= 7

PWD = $(shell pwd)
teeny_usb_desc.c teeny_usb_desc.h:$(TEENYDT)
	@echo generate USB descriptor
	cd $(PWD)/../../TeenyDT && \
	lua gen_descriptor.lua $(PWD)/$< -maxmem=$(USBMEM) -maxep=$(USBEP) && \
	cd $(PWD)

clean:
	$(RM) $(OUTDIR)/* $(OBJDIR)/*
clear:
	$(RM) $(OUTDIR) obj

.PHONY: all clean clear

-include $(DEPENDS)
