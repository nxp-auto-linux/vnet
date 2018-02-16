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
* LS1043 to CM7-FPGA
* RRM Emu platform

To build virtual ethernet driver choose NXP_VETH config.

### Virtual Ethernet for BlueBox-Mini
To build virtual ethernet for BlueBox-Mini platform choose NXP\_VETH\_BBMINI 
configuration choice.

### Virtual Ethernet for LS1043 to CM7-FPGA
To build virtual ethernet for LS1043 to CM7-FPGA platform choose 
NXP\_VETH\_LS1043\_CM7FPGA configuration choice.

### Virtual Ethernet for RRM Emu platform
