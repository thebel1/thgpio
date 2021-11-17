# Note: all the source code contained in this repository is for educational use only, and is provided with no support, endorsement, warranty, guarantee or implied fitness for a particular purpose. It does not constitute sample driver code. Do not reach out to VMware to support this code. Do not copy and reuse in any commercial or product setting.

# thgpio
Native ESXi driver for RPi 4's GPIO interface.

Allows controlling the pins of the Raspberry Pi 4's GPIO interface.

Still very much work-in-progress!

Branch main contains:
- the base GPIO hardware driver
- a Python library for interacting with the GPIO MMIO memory directly
- a Python utility for interacting with the GPIO interface
- a Python utility for interacting with the Pimoroni FanShim

### Videos

These demos use the Pimoroni FanShim as an example. Any GPIO accessory should work analogously.

Branch main: https://youtu.be/Mw8L1XAi8qA?t=170

Branch fanShimButton: https://youtu.be/YjSxgKr7gZg

Branch charDev: https://youtu.be/Ri1py3AzIsU?t=107

### Installation

To install the driver, download the VIB file in the build directory and copy it to your RPi.

Install it as described here: https://kb.vmware.com/s/article/2008939

Note: you will need to reboot the RPi after installing the VIB.

I would recommend downloading the Python library in ./pyUtil/gpioLib/ as well so you don't have to worry about all the bit twiddling.

### Post-Installation Setup

You may need to manually change the `GPIO_DEVICE_PATH` variable in `./pyUtil/gpioLib/__init__.py` to reflect the correct name for the character device. This is due to a limitation in the VMKernel character device API.

1. Open the `__init__.py` file ( https://github.com/thebel1/thgpio/blob/main/pyUtil/gpioLib/__init__.py )
2. Locate the line `GPIO_DEVICE_PATH = '/dev/vmgfx32'`
3. Run the command `ls /dev/` in the ESXi shell and locate the vmgfx* device(s)
4. Note the device number corresponding to the GPIO device
4.1. If you installed the GPIO driver first, the file path should be `/dev/vmgfx32`
4.2. If you installed the GPIO driver second, the file should be `/dev/vmgfx33`
4.3. It just keeps counting up from there as far as I can tell
5. Go back to the `__init__.py` file and set `GPIO_DEVICE_PATH` to the correct device path
6. Save the file and you're set
