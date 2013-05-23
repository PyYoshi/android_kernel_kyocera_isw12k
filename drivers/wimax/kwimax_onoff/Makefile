##################################################################

KERNEL_DIR ?= $(ANDROID_BUILD_TOP)/../kernel
KERNEL_OUT ?= $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ

INSTALL_DIR ?= $(ANDROID_PRODUCT_OUT)/system/lib/modules

obj-m := kwimax_onoff.o

MAKE_FLAGS := -C $(KERNEL_DIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
EXTRA_CFLAGS += $(addprefix -I, $(INC_PATH))
#EXTRA_CFLAGS += -DKWIMAX_ONOFF_DEBUG

# FEATURE_KCC_WIMAX_BOOT_APP_A   2011.08.29
EXTRA_CFLAGS += -DFEATURE_KCC_WIMAX_BOOT_APP

# FEATURE_KCC_WIMAX_CHG_PAFAULT_PORT A  2012.03.13
EXTRA_CFLAGS += -DFEATURE_KCC_WIMAX_CHG_PAFAULT_PORT

##################################################################
