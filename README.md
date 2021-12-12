# RPMsg Virtual Ethernet Network Driver

This Linux kernel module exchanges Ethernet frames over the RPMsg bus, providing
a virtual point-to-point network connection between two cores. The virtual
network interface is named `rpmsgN`, where N starts at 0.

This driver is useful to establish a virtual network connection between two
heterogenous cores under the `rpmsg` framework, one running Linux, and the other
typically running an RTOS. The RTOS requires a matching driver, which is not
provided here.

## How to build

To perform an out-of-tree build, run `make`. The resulting kernel module will be
called `rpmsg_net.ko`.

If your kernel's build directory is in a non-standard location, you can specify
it with `make KERNEL_SRC=...`. If cross-compiling, please be sure to source your
cross-compiler's environment, first.

## Caveats

The default size for MAX_RPMSG_BUF_SIZE in the Linux kernel is 512 bytes. This
driver assumes that this value has been increased to 2048, to accommodate the
more typical 1500-byte MTU of Ethernet frames. See `virtio_rpmsg_bus.c` in your
kernel's sources. You will need to modify it, and recompile the rpmsg driver.

## TODO

There are a few open issues with this driver. Please see TODOs marked in
`rpmsg_net.c` for more information.
