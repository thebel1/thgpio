#!/usr/bin/python
################################################################################
# UW interface to the GPIO MMIO area.
# 
# Tom Hebel, 2020
################################################################################

import sys
import argparse
from gpioLib import *

#########################################################################

class GPIOCommands:
    gpio = GPIO()

    #########################################################################
    # infoCmd --
    #
    #   Get information about the GPIO interface, such as the MMiO area size.
    #########################################################################
    def infoCmd(self, args):
        print('\nAll registers are 4 bytes long.')
        print('All pins are either set (1) or cleared (0).')
        print(('\nRegisters:\n'
            + '\n'.join([x for x in gpioRegisters.keys()])))
        print(('\nFunction Selectors:\n'
            + '\n'.join([x for x in gpioFuncSelectors.keys()])))
        print(('\nPull-up/down States:\n'
            + '\n'.join([x for x in gpioPUDs.keys()])))

    #########################################################################
    # readRegCmd --
    #
    #   Command func for reading register value.
    #########################################################################
    def readRegCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio read',
                                        description=('Read the value of a GPIO'
                                                    ' register'))
        parser.add_argument('register',
                            nargs=1,
                            type=str,
                            help='the GPIO register to read')
        args = parser.parse_args(argv)
        reg = str(args.register[0])
        if reg not in gpioRegisters.keys():
            print('Invalid register: {}'.format(reg))
            exit(1)
        print(self.gpio.readReg(reg))

    #########################################################################
    # writeRegCmd --
    #
    #   Command func for writing register value.
    #########################################################################
    def writeRegCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio write',
                                        description=('Write 4 bytes to a GPIO'
                                                    ' register'))
        parser.add_argument('register',
                            nargs=1,
                            type=str,
                            help='the GPIO register to write to')
        parser.add_argument('value',
                            nargs=1,
                            type=int,
                            help='the 4-byte value to write')
        args = parser.parse_args(argv)
        reg = str(args.register[0])
        val = int(args.value[0])
        if val > GPIO_INT_MAX:
            print('Value doesn\'t fit into 4 bytes: {}'.format(val))
            exit(1)
        if reg not in gpioRegisters.keys():
            print('Invalid register: {}'.format(reg))
            exit(1)
        self.gpio.writeReg(reg, val)

    #########################################################################
    # funcSelPinCmd --
    #
    #   Command func for setting the function of a GPIO pin.
    #########################################################################
    def funcSelPinCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio fsel',
                                        description=('sets the function of a GPIO'
                                                    ' pin'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        parser.add_argument('func',
                            nargs=1,
                            type=str,
                            help='the function to select')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        func = str(args.func[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        if func not in gpioFuncSelectors.keys():
            print('Invalid function: {}'.format(func))
            exit(1)
        self.gpio.funcSelPin(pin, func)

    #########################################################################
    # setPinCmd --
    #
    #   Command func for setting a GPIO pin to 1.
    #########################################################################
    def setPinCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio set',
                                        description=('sets a GPIO pin to 1'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        self.gpio.setPin(pin)

    #########################################################################
    # clrPinCmd --
    #
    #   Command func for clearing a GPIO pin.
    #########################################################################
    def clrPinCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio clear',
                                        description=('clears a GPIO pin'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        self.gpio.clrPin(pin)

    #########################################################################
    # levPinCmd --
    #
    #   Command func for getting the level of a GPIO pin.
    #########################################################################
    def levPinCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio level',
                                        description=('gets the level of a GPIO'
                                                     ' pin'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        print(self.gpio.levPin(pin))

    #########################################################################
    # pollPinCmd --
    #
    #   Command func for getting the level of a GPIO pin.
    #########################################################################
    def pollPinCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio poll',
                                        description=('polls the level of a GPIO'
                                                     ' pin until it changes'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        print(self.gpio.pollPin(pin))

    #########################################################################
    # getPullCmd --
    #
    #   Command func for getting the pull of a GPIO pin.
    #########################################################################
    def getPullCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio getpull',
                                        description=('gets the pull of a GPIO'
                                                     ' pin'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        print(self.gpio.getPull(pin))

    #########################################################################
    # setPullCmd --
    #
    #   Command func for setting the pull of a GPIO pin.
    #########################################################################
    def setPullCmd(self, argv):
        parser = argparse.ArgumentParser(prog='gpio setpull',
                                        description=('sets the pull of a GPIO'
                                                    ' pin'))
        parser.add_argument('pin',
                            nargs=1,
                            type=int,
                            help='the GPIO pin')
        parser.add_argument('pull',
                            nargs=1,
                            type=str,
                            help='the pull to apply (UP, DN, OFF)')
        args = parser.parse_args(argv)
        pin = int(args.pin[0])
        pull = str(args.pull[0])
        if pin > GPIO_MAX_PIN:
            print('Invalid pin: {}'.format(pin))
            exit(1)
        if pull not in gpioPUDs.keys():
            print('Invalid pull value: {}'.format(pull))
            exit(1)
        self.gpio.setPull(pin, pull)

#########################################################################
# main --
#
#   Main entry point.
#########################################################################
def main(argv):
    cmdsObj = GPIOCommands()
    commands = {
        'info': cmdsObj.infoCmd,
        'read': cmdsObj.readRegCmd,
        'write': cmdsObj.writeRegCmd,
        'fsel': cmdsObj.funcSelPinCmd,
        'set': cmdsObj.setPinCmd,
        'clear': cmdsObj.clrPinCmd,
        'level': cmdsObj.levPinCmd,
        'poll': cmdsObj.pollPinCmd,
        'getpull': cmdsObj.getPullCmd,
        'setpull': cmdsObj.setPullCmd,
    }
    progDesc = ('Wrapper for the GPIO device driver.'
                '\nFor more information, check out the documenation for the'
                ' RPi 4B\'s system on a chip, BCM2711:'
                '\nhttps://www.raspberrypi.org/documentation/hardware/'
                'raspberrypi/bcm2711/rpi_DATA_2711_1p0.pdf'
                '\n\nRun `./self.gpio.py info` to get information about the GPIO'
                ' interface.'
                '\n\nAll registers are 4 bytes long.'
                '\n\nCommands:'
                '\nread\t<register>'
                '\nwrite\t<register> <value>'
                '\nfsel\t<pin> <func>'
                '\nset\t<pin>'
                '\nclear\t<pin>'
                '\nlevel\t<pin>'
                '\npoll\t<pin>'
                '\ngetpull\t<pin>'
                '\nsetpull\t<pin> <pull>')
    parser = argparse.ArgumentParser(prog='gpio',
                                     description=progDesc,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('command', help='interact with the GPIO interface')
    parser.add_argument('args',
                        nargs='*',
                        help='arguments for the command')
    args = parser.parse_args()
    if args.command not in commands.keys():
        print('Invalid command: {}'.format(args.command))
        exit(1)
    # Call the command func
    commands[args.command](args.args)

if __name__ == '__main__':
    main(sys.argv)