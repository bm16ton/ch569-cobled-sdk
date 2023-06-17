COMPILER_PREFIX ?= riscv-none-elf


BOARD_PATH = ../boards/ch565w_evk/
include $(BOARD_PATH)/board.mk
BOARD_SRC = ../boards/ch565w_evk/
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
SOURCES  += $(wildcard ../boards/ch565w_evk/*.c)
SOURCES  += ../../driver_ch56x/ch56xlib/Peripheral/src/CH56x_clk.c
SOURCES  += ../../driver_ch56x/ch56xlib/Peripheral/src/CH56x_sys.c
SOURCES  += ../../driver_ch56x/ch56xlib/Peripheral/src/CH56x_timer.c
INCLUDES += $(TUSB_PATH)/../class
INCLUDES += $(TUSB_PATH)/../class/user
SOURCES  += ../../driver_ch56x/ch56xlib/Startup/startup_CH56x.S
INCLUDES += $(TUSB_PATH)../driver_ch56x/ch56xlib/Peripheral/inc/
INCLUDES += $(TUSB_PATH)/../driver_ch56x
INCLUDES += ../boards/ch565w_evk
INCLUDES += $(TUSB_PATH)/
INCLUDES += ./
INCLUDES += $(TUSB_PATH)/../driver_ch56x/ch56xlib/RVMSIS
INCLUDES += ../../driver_ch56x/ch56xlib/Peripheral/inc

# MCU driver source
SOURCES += $(wildcard $(TUSB_PATH)/../driver_ch56x/*.c)

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

CPU ?=		rv32imac_zicsr
CCFLAGS     += -march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8 $(OPTIM_OPTS) -fmessage-length=0 -fsigned-char
CCFLAGS     += $(addprefix -D, $(DEFINES))
CCFLAGS     += $(addprefix -I, $(INCLUDES))
CCFLAGS     += -ffunction-sections -fdata-sections
LDFLAGS     += $(CCFLAGS)
LDFLAGS     += -Wl,--script='$(LDSCRIPT)' -Wl,-Map=$(OUTDIR)/$(TARGET).map  -std=gnu99 -MMD -MP -MT"$(@)"
LDFLAGS     += -Wl,--gc-sections  -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs


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

USBMEM ?= 1024
USBEP  ?= 7

clean:
	$(RM) $(OUTDIR)/* $(OBJDIR)/*
clear:
	$(RM) $(OUTDIR) obj

.PHONY: all clean clear

-include $(DEPENDS)
