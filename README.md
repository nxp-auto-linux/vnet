# Linux PCIe Virtual Device Driver
This is a virtual communication device driver over PCIe.
It can be build as a virtual Ethernet driver or a virtual TTY driver.
The current implementation supports virtual Ethernet only.

## Virtual Ethernet Driver
This is a driver implementing a virtual Ethernet interface to enable communication
over PCIe with a PCI endpoint device that can also implement a virtual Ethernet 
interface.

The following platforms are supported by virtual ethernet driver:

* BlueBox-Mini (LS2084 to S32V234)
* LS1043 to S32G-FPGA
* RRM Emu platform

To build virtual ethernet driver choose NXP_VETH config.

### Virtual Ethernet for BlueBox-Mini
Virtual Ethernet for BlueBox-Mini platform enables Ethernet like communication
over PCIe between LS2084A and S32V234 processors located on BlueBox-Mini board.

To build virtual ethernet for BlueBox-Mini platform choose NXP\_VETH\_BBMINI 
configuration choice.

### Virtual Ethernet for LS1043 to S32G-FPGA
Virtual Ethernet for LS1043 to S32G-FPGA platform enables Ethernet like communication
over PCIe between LS1043 processor and a S32G processor partially emulated on an
FPGA board (Cortex-M7 cores plus some peripherals).

To build virtual ethernet for LS1043 to S32G-FPGA platform choose
NXP\_VETH\_LS1043\_S32GFPGA configuration choice.

### Virtual Ethernet for RRM Emu platform
