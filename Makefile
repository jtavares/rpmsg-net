
obj-m += rpmsg_net.o

KERNEL_SRC  := /lib/modules/$(shell uname -r)/build

PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules
	
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

