# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

# ------------------------------------------------
# Family: Miromico FMLR Modules

ifneq (,$(filter fmlr_72_x_stl0,$(FAMILIES)))
    MCU		:= stm32l0
    LD_SCRIPTS	+= $(BL)/src/arm/stm32lx/ld/STM32L0xxB.ld
    CFLAGS	+= -Wa,-adhlns="$@.lst"
    DEFS	+= -DSTM32L0 -DSTM32L071xx
    DEFS	+= -DCFG_fmlr_72_x_stl0_board
    DEFS	+= -DBRD_IMPL_INC='"fmlr-72-x-stl0.h"'
    OOCFGS	+= $(TOOLSDIR)/openocd/wsms130.cfg
    JLINKDEV	?= STM32L071RB
    BL_BRD	:= FMLR-72-X-STL0
endif

ifneq (,$(filter fmlr_61_x_stl0,$(FAMILIES)))
    MCU		:= stm32l0
    LD_SCRIPTS	+= $(BL)/src/arm/stm32lx/ld/STM32L0xxZ.ld
    CFLAGS	+= -Wa,-adhlns="$@.lst"
    DEFS	+= -DSTM32L0 -DSTM32L071xx
    DEFS	+= -DCFG_fmlr_61_x_stl0_board
    DEFS	+= -DBRD_IMPL_INC='"fmlr-6x-x-stl0.h"'
    OOCFGS	+= $(TOOLSDIR)/openocd/wsms130.cfg
    JLINKDEV	?= STM32L071RZ
    BL_BRD	:= FMLR-72-X-STL0
endif

ifneq (,$(filter fmlr_61_x_ma625,$(FAMILIES)))
    MCU		:= max32625
	LD_SCRIPTS += $(HALDIR)/memory.ld
    CFLAGS	+= -Wa,-adhlns="$@.lst"
    DEFS	+= -DMAX32625
    DEFS	+= -DTARGET=MAX32625
    DEFS	+= -DTARGET_REV=0x4132
    DEFS	+= -DCFG_fmlr_61_x_ma625
    DEFS	+= -DBRD_IMPL_INC='"fmlr-6x-x-ma625.h"'
    JLINKDEV	?= MAX32625IWY
    OOCFGS	+= $(TOOLSDIR)/openocd/max32625.cfg
    BL_BRD	:= FMLR-60-X-MAX32625
endif
