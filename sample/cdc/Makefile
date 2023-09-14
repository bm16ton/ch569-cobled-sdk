COMPILER_PREFIX ?= riscv-none-elf
BOARD ?= ch565w_evk
TARGET = usbcdc
BOARD_PATH = ../boards/ch565w_evk/
BOARD_SRC = ../boards/ch565w_evk/

# project source

SOURCES  += Main.c \
	syscalls.c \
	tft.c \
	cdc.c

INCLUDES += .
DEFINES  = __forceinline=inline NO_HOST
# board source
SOURCES  += $(BOARD_SRC)
INCLUDES += $(BOARD_INC)
DEFINES  += $(BOARD_DEF)

# Teeny USB source
SOURCES  += $(wildcard ../boards/ch565w_evk/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/rvmsis/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/drv/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/board/gen/*.c)
SOURCES  += $(wildcard ../../wch-ch56x-bsp/usb/usb_devbulk/*.c)
INCLUDES += ../../wch-ch56x-bsp/rvmsis
INCLUDES += ../boards/ch565w_evk
INCLUDES += ./
INCLUDES += ../../wch-ch56x-bsp/board/gen
INCLUDES += ../../wch-ch56x-bsp/drv
INCLUDES += ../../wch-ch56x-bsp/usb/usb_devbulk

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
CCFLAGS		+= -DDEBUG -DDebug_UART1 -DDEBUG_USB3_REQ -DDEBUG_USB2_REQ -DDEBUG_USB3_EP
CCFLAGS     += -ffunction-sections -fdata-sections
LDFLAGS     += $(CCFLAGS)
LDFLAGS     += -Wl,--script=../../wch-ch56x-bsp/ld/Link.ld -Wl,-Map=$(OUTDIR)/$(TARGET).map  -std=gnu99 -MMD -MP -MT"$(@)"
LDFLAGS     += -Wl,--gc-sections -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs -specs=nosys.specs


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
	@echo Building USB-CDC example
	@$(MAKE) demo TARGET=usbcdc BOARD='ch565w_evk'


demo: $(OUTDIR) $(OUTDIR)/$(TARGET).bin $(OUTDIR)/$(TARGET).hex

$(OBJDIR)/%.o:%.c
	@echo building $<
	@$(CC) $(CCFLAGS) -c $< -o $@ -lm
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
	@$(LD) $(LDFLAGS) $(OBJECTS) -o $@ -lm
	@$(SIZE) $@

USBMEM ?= 4096
USBEP  ?= 7

clean:
	rm -Rf $(OUTDIR)/* $(OBJDIR)/*
	rm -Rf ./obj/gud
clear:
	$(RM) $(OUTDIR) obj

.PHONY: all clean clear

-include $(DEPENDS)
