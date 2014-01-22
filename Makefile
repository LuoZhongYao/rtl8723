MODULE_NAME := wb
ifneq ($(KERNELRELEASE),)
#$(MODULE_NAME)-y += \
	usb.o

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-objs := drv.o queue.o patch.o
else
PWD := $(shell pwd)
KSRC := /home/lzy/build/linux-3.0.8
KVER := 3.0.8
KDIR := $(KSRC)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	adb push $(MODULE_NAME).ko /data/goc/

clean:
	-rm -rf *.o *.mod.c *.mod.o *.ko *.symvers *.order *.a  tags GPATH GRTAGS GTAGS
endif
