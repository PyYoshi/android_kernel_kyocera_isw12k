######################################################################
# 
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2011 KYOCERA Corporation
# (C) 2012 KYOCERA Corporation
# 
######################################################################
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
MODULES = kc_sdgdrv
obj-m := $(MODULES).o
$(MODULES)-objs += sdgpiodrv.o

MAKE_FLAGS := -C $(KERNEL_DIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
EXTRA_CFLAGS += $(addprefix -I, $(INC_PATH))

EXTRA_CFLAGS += -D_LINUX_KERNEL_

ifeq ($(SDAS_DBG), ON)
EXTRA_CFLAGS += -DSDASDBG
endif

ifeq ($(KERNEL_DBG), ON)
EXTRA_CFLAGS += -D_KERNEL_DBG_
endif

