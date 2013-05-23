##################################################################

KERNEL_DIR ?= $(ANDROID_BUILD_TOP)/../kernel
KERNEL_OUT ?= $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ

INSTALL_DIR ?= $(ANDROID_PRODUCT_OUT)/system/lib/modules

EXTRA_LDFLAGS := -s
#EXTRA_CFLAGS += -DFEATURE_KCC_WIMAX_DBG_MSG_SDIOWPR

bcmsdio-objs	:= bcm_sdio.o

obj-m    := bcmsdio.o


MAKE_FLAGS := -C $(KERNEL_DIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
EXTRA_CFLAGS += $(addprefix -I, $(INC_PATH))

##################################################################
