CPU = zicsr

DEV_TYPE = ilp32
BOARD_INC = $(BOARD_PATH)  $(TUSB_PATH)/../driver_ch56x/ch56xlib/Peripheral/inc/
BOARD_SRC = $(BOARD_PATH)/board.c
BOARD_SRC += $(TUSB_PATH)/../driver_ch56x/ch56xlib
LDSCRIPT = $(TUSB_PATH)/../driver_ch56x/ch56xlib/Ld/Link.ld

