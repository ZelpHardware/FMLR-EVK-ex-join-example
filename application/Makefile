TARGET := fmlr_72_x_stl0
#TARGET := fmlr_61_x_ma625

VARIANTS := eu868 #us915 hybrid simul

REGIONS.simul := eu868

TARGET.simul := unicorn

CFLAGS += -Os
CFLAGS += -g
CFLAGS += -Wall -Wno-main

SVCS += app

ifeq ($(TARGET),fmlr_61_x_ma625)
DEFS += -DPERIPH_TRNG
DEFS += -DADC_AS_TRNG
DEFS += -DBRD_DEV_NONCE_WITH_RANDOM_GENERATOR
DEFS += -DSET_DIO3_AS_TCXO_CTRL
DEFS += -DREGMODE_IS_LDO
DEFS += -DSHUTDOWN_SPI_IF_NOT_SELECTED
DEFS += -DMINRX_SYMS=20
else
DRIVER += mx25.c
endif
DRIVER += sht21.c
DRIVER += sht3x.c

DEFS += -DDEBUG_RX
DEFS += -DDEBUG_TX

LMICCFG += eeprom_keys
LMICCFG += DEBUG
LMICCFG += extapi
LMICCFG	+= EXTI_IRQ_HANDLER=uexti_handler

JLINK = /mnt/c/Program\ Files\ \(x86\)/SEGGER/JLink/JLink.exe

include ../extensions/makefiles/application.gmk
