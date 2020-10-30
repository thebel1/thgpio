################################################################################
# UW library interfacing with the GPIO MMIO area.
# 
# Tom Hebel, 2020
################################################################################

import math
import struct
import fcntl

#########################################################################

GPIO_DEVICE_PATH = '/dev/vmgfx32'
GPIO_REG_SIZE = 4
GPIO_MAX_PIN = 57
GPIO_MMIO_SIZE = 248
GPIO_INT_MAX = pow(2, 32) - 1
GPIO_PIN_MASK = 31
GPIO_RPI_BYTE_ORDER = 'little'
GPIO_IOCTL_COOKIE_SIZE = 8

#########################################################################

#
# GPIO ioctl commands
#
GPIO_IOCTL_POLL = 1

#########################################################################

#
# GPIO function selectors
#
gpioFuncSelectors = {
    'IN':  0b000,
    'OUT': 0b001,
    'F0':  0b100,
    'F1':  0b101,
    'F2':  0b110,
    'F3':  0b111,
    'F4':  0b011,
    'F5':  0b010,
}
gpioPinToSelReg= [
   'GPFSEL0', 'GPFSEL1', 'GPFSEL2', 'GPFSEL3', 'GPFSEL4', 'GPFSEL5',
]

#
# Pull-up/down states
#
gpioPUDs = {
    'OFF': 0b00,
    'DN':  0b10,
    'UP':  0b01,
}
gpioPinToPudReg = [
   'GPIO_PUP_PDN_CNTRL_REG0',
   'GPIO_PUP_PDN_CNTRL_REG1',
   'GPIO_PUP_PDN_CNTRL_REG2',
   'GPIO_PUP_PDN_CNTRL_REG3',
]

#
# GPIO registers
#
gpioRegisters = {
    #
    # Function selector registers which control the operation of the 54 gpio pins.
    #
    'GPFSEL0':  0x00,       # gpio function select 0
    'GPFSEL1':  0x04,       # gpio function select 1
    'GPFSEL2':  0x08,       # gpio function select 2
    'GPFSEL3':  0x0c,       # gpio function select 3
    'GPFSEL4':  0x10,       # gpio function select 4
    'GPFSEL5':  0x14,       # gpio function select 5

    #
    # Used to set a specific gpio pin. Setting to 0 has no effect.
    #
    'GPSET0':   0x1c,       # gpio pin output set 0
    'GPSET1':   0x20,       # gpio pin output set 1

    #
    # Used to clear gpio pin. Setting to 0 has no effect.
    #
    'GPCLR0':   0x28,       # gpio pin output clear 0
    'GPCLR1':   0x2c,       # gpio pin output clear 1

    #
    # Return value of a pin.
    #
    'GPLEV0':   0x34,       # gpio pin level 0
    'GPLEV1':   0x38,       # gpio pin level 1

    #
    # Used to retrieve signal edge detection results. 
    #
    'GPEDS0':   0x40,       # gpio pin event detect status 0 
    'GPEDS1':   0x44,       # gpio pin event detect status 1 
    'GPREN0':   0x4c,       # gpio pin rising edge detect enable 0 
    'GPREN1':   0x50,       # gpio pin rising edge detect enable 1 
    'GPFEN0':   0x58,       # gpio pin falling edge detect enable 0 
    'GPFEN1':   0x5c,       # gpio pin falling edge detect enable 1 
    'GPHEN0':   0x64,       # gpio pin high detect enable 0 
    'GPHEN1':   0x68,       # gpio pin high detect enable 1 
    'GPLEN0':   0x70,       # gpio pin low detect enable 0 
    'GPLEN1':   0x74,       # gpio pin low detect enable 1 
    'GPAREN0':  0x7c,       # gpio pin async rising edge detect 0 
    'GPAREN1':  0x80,       # gpio pin async rising edge detect 1 
    'GPAFEN0':  0x88,       # gpio pin async falling edge detect 0
    'GPAFEN1':  0x8c,       # gpio ping async falling edge detect 1 

    #
    # Pull-up/down config.
    #
    'GPIO_PUP_PDN_CNTRL_REG0': 0xe4,
    'GPIO_PUP_PDN_CNTRL_REG1': 0xe8,
    'GPIO_PUP_PDN_CNTRL_REG2': 0xec,
    'GPIO_PUP_PDN_CNTRL_REG3': 0xf0,
}

#########################################################################
# GPIO --
#
#   GPIO char dev interface class.
#########################################################################
class GPIO:
    gpioDev = open(GPIO_DEVICE_PATH, 'r+b')
    def __del__(self):
        self.gpioDev.close()

    #########################################################################
    # pollReg --
    #
    #   Polls a GPIO register until the masked bits have changed. Then,
    #   returns the registers current value.
    #########################################################################
    def pollReg(self, reg, mask):
        try:
            #
            # The GPIO driver's ioctl struct looks like this:
            #
            # +--------------------+
            # |  uint16_t offset   |
            # +--------------------+
            # | uint32_t mask/data |
            # +--------------------+
            # |  uint16_t padding  |
            # +--------------------+
            #
            ioctlData = bytearray(struct.pack('<HIH',
                                              gpioRegisters[reg],
                                              mask,
                                               0))
            fcntl.ioctl(self.gpioDev, GPIO_IOCTL_POLL, ioctlData, 1)
            # Data is passed back in the mask/data field
            out = struct.unpack('<HIH', ioctlData)[1]
        except Exception as e:
            # Typically, a timeout will have occurred
            return None
        return out

    #########################################################################
    # readReg --
    #
    #   Read 4-byte GPIO register value.
    #########################################################################
    def readReg(self, reg):
        try:
            self.gpioDev.seek(gpioRegisters[reg])
            val = self.gpioDev.read(GPIO_REG_SIZE)
            word = int.from_bytes(val, GPIO_RPI_BYTE_ORDER)
        except Exception as e:
            print('Read failed: {}'.format(e))
            exit(1)
        return word

    #########################################################################
    # writeReg --
    #
    #   Write 4-byte value to a GPIO register.
    #########################################################################
    def writeReg(self, reg, val):
        try:
            self.gpioDev.seek(gpioRegisters[reg])
            self.gpioDev.write(val.to_bytes(4, GPIO_RPI_BYTE_ORDER))
        except Exception as e:
            print('Write failed: {}'.format(e))
            exit(1)

    #########################################################################
    # funcSelPin --
    #
    #   Set the function of a GPIO pin.
    #########################################################################
    def funcSelPin(self, pin, func):
        reg = gpioPinToSelReg[int(math.floor(pin / 10))]
        shift = ((pin % 10) * 3)
        origVal = self.readReg(reg)
        newVal = origVal | (0b111 << shift)
        newVal ^= (0b111 << shift)
        newVal |= (gpioFuncSelectors[func] << shift)
        try:
            self.gpioDev.seek(gpioRegisters[reg])
            self.gpioDev.write(newVal.to_bytes(4, GPIO_RPI_BYTE_ORDER))
        except Exception as e:
            print('Function select failed: {}'.format(e))
            exit(1)

    #########################################################################
    # setPin --
    #
    #   Set a GPIO pin's value to 1.
    #########################################################################
    def setPin(self, pin):
        if pin < 32:
            self.writeReg('GPSET0', 1 << pin)
        else:
            self.writeReg('GPSET1', 1 << (pin & GPIO_PIN_MASK))

    #########################################################################
    # clrPin --
    #
    #   Clears GPIO pin's value.
    #########################################################################
    def clrPin(self, pin):
        if pin < 32:
            self.writeReg('GPCLR0', 1 << pin)
        else:
            self.writeReg('GPCLR1', 1 << (pin & GPIO_PIN_MASK))

    #########################################################################
    # levPin --
    #
    #   Returns a GPIO pin's level (0 or 1).
    #########################################################################
    def levPin(self, pin):
        if pin < 32:
            val = self.readReg('GPLEV0')
        else:
            val = self.readReg('GPLEV1')
        if (val & (1 << (pin & GPIO_PIN_MASK))) != 0:
            out = 1
        else:
            out = 0
        return out

    #########################################################################
    # pollPin --
    #
    #   Polls a pin's value until it changes and then returns its new value.
    #########################################################################
    def pollPin(self, pin, prev):
        if pin < 32:
            mask = 1 << pin
            cur = self.pollReg('GPLEV0', mask)
        else:
            mask = 1 << (pin & GPIO_PIN_MASK)
            cur = self.pollReg('GPLEV1', mask)
        if cur is not None:
            if (cur & (1 << (pin & GPIO_PIN_MASK))) != 0:
                out = 1
            else:
                out = 0
        else:
            return prev
        return out

    #########################################################################
    # getPull --
    #
    #   Get pull-up/down state of a pin.
    #########################################################################
    def getPull(self, pin):
        shift = (pin % 16) * 2
        reg = gpioPinToPudReg[int(math.floor(pin / 16))]
        pud = self.readReg(reg)
        mask = (0b11 << shift)
        key = (pud & mask) >> shift
        return list(gpioPUDs.keys())[list(gpioPUDs.values()).index(key)]

    #########################################################################
    # setPull --
    #
    #   Set pull-up/down state of a pin.
    #########################################################################
    def setPull(self, pin, pull):
        pud = gpioPUDs[pull]
        shift = (pin % 16) * 2
        reg = gpioPinToPudReg[int(math.floor(pin / 16))]
        oldPud = self.readReg(reg)
        newPud = oldPud | (0b11 << shift)
        newPud ^= (0b11 << shift)
        newPud |= (pud << shift)
        self.writeReg(reg, newPud)