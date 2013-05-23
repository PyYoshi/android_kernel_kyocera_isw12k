ifeq ($(TARGET_BUILD_VARIANT),user)
CFLAGS_ram_console.o += -DKC_IGNORE_VERBOSE
endif

obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o
obj-$(CONFIG_ANDROID_LOGGER)		+= logger.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_console.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder_ioctl_convert.o

$(obj)/binder_ioctl_convert.c: $(src)/binder_ioctl_convert.c.head $(obj)/binder_ioctl_convert.c.mid $(src)/binder_ioctl_convert.c.tail
	cat $^ > $@

$(obj)/binder_ioctl_convert.c.mid: $(src)/binder.h
	-rm $@
	grep -E "^#define[[:space:]]+BINDER[0-9A-Z_]+[[:space:]]+_IO[A-Z_]*\(.*\)$$" \
		$< | awk {'print "\t{"$$2 ", \"" $$2 "\"}," '} >> $@
