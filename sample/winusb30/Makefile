COMPILER_PREFIX ?= riscv-none-elf
BOARD ?= ch565w_evk
TARGET = winusb30
BOARD_PATH = ../boards/ch565w_evk/

BOARD_SRC = ../boards/ch565w_evk/
# TeenyDT format descriptor
TEENYDT = winusb30_desc.lua
USBMEM = 4096
USBEP = 16
# project source

SOURCES  += winusb30.c \
           teeny_usb_desc.c \
           syscalls.c

INCLUDES += .
DEFINES  = __forceinline=inline NO_HOST
# board source
SOURCES  += $(BOARD_SRC)
INCLUDES += $(BOARD_INC)
DEFINES  += $(BOARD_DEF)

# Teeny USB source
TUSB_PATH = ../../core
SOURCES  += $(wildcard $(TUSB_PATH)/*.c)
SOURCES  += $(wildcard ../boards/ch565w_evk/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/rvmsis/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/drv/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/board/gen/*.c)
SOURCES  += $(wildcard ../../class/*.c)
SOURCES  += $(wildcard ../../driver_ch56x/*.c)
SOURCES  += $(wildcard ../../class/cdc/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/usb/usb_devbulk/*.c)
INCLUDES += ../../class
INCLUDES += ../../class/user
INCLUDES += ../../wch-ch56x-bsp/rvmsis
INCLUDES += ../../driver_ch56x
INCLUDES += ../boards/ch565w_evk
INCLUDES += ./
INCLUDES += ../../core
INCLUDES += ../../wch-ch56x-bsp/board/gen
INCLUDES += ../../class/cdc
INCLUDES += ../../wch-ch56x-bsp/drv
INCLUDES += ../../wch-ch56x-bsp/usb/usb_devbulk
INCLUDES += ../../driver_ch56x/ch56xlib/Peripheral/inc

# MCU driver source

SOURCES  += ../../wch-ch56x-bsp/startup/startup_CH56x.S
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

CPU ?=
CCFLAGS     += -march=rv32imac_zicsr -mabi=ilp32  -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g
CCFLAGS     += $(addprefix -D, $(DEFINES))
CCFLAGS     += $(addprefix -I, $(INCLUDES))
CCFLAGS		+= -DDEBUG -DDebug_UART1 -DDEBUG_USB3_REQ -DDEBUG_USB2_REQ
CCFLAGS     += -ffunction-sections -fdata-sections
LDFLAGS     += $(CCFLAGS)
LDFLAGS     += -Wl,--script=../../wch-ch56x-bsp/ld/Link.ld -Wl,-Map=$(OUTDIR)/$(TARGET).map  -std=gnu99 -MMD -MP -MT"$(@)"
LDFLAGS     += -Wl,--gc-sections  -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs -specs=nosys.specs


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

USBMEM ?= 4096
USBEP  ?= 7

desc: teeny_usb_desc.c


#PWD = $(shell pwd)
#teeny_usb_desc.h teeny_usb_desc.c:$(TEENYDT)
#	@echo generate USB descriptor
#	cd $(PWD)/../../TeenyDT && \
#	lua gen_descriptor.lua $(PWD)/$< -maxmem=$(USBMEM) -maxep=$(USBEP) && \
#	cd $(PWD)


clean:
	rm -Rf $(OUTDIR)/* $(OBJDIR)/*
clear:
	$(RM) $(OUTDIR) obj

.PHONY: all clean clear

-include $(DEPENDS)
