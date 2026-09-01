# SPDX-License-Identifier: GPL-2.0-only
obj-m += aorus-wmi.o

# KUnit suite: builds only on kernels with KUnit support enabled.
ifneq ($(CONFIG_KUNIT),)
obj-m += aorus-wmi-test.o
CFLAGS_aorus-wmi-test.o = -DAORUS_WMI_KUNIT_TEST
endif

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
