#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2012-2020, Alan Aguiar <alanjas@hotmail.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

import os
import sys
import subprocess

from gettext import gettext as _
from plugins.plugin import Plugin

from TurtleArt.tapalette import make_palette
from TurtleArt.tapalette import palette_name_to_index
from TurtleArt.tapalette import palette_blocks
from TurtleArt.tapalette import special_block_colors
from TurtleArt.talogo import logoerror
from TurtleArt.taconstants import CONSTANTS
from TurtleArt.taprimitive import Primitive, ArgSlot, ConstantArg
from TurtleArt.tatype import TYPE_INT, TYPE_FLOAT, TYPE_STRING, TYPE_NUMBER

sys.path.insert(0, os.path.abspath('./plugins/arduino'))
import pyfirmata

VALUE = {_('HIGH'): 1, _('LOW'): 0}
MODE = {_('INPUT'): pyfirmata.INPUT, _('OUTPUT'): pyfirmata.OUTPUT,
        _('PWM'): pyfirmata.PWM, _('SERVO'): pyfirmata.SERVO}

ERROR = _('ERROR: Check the Arduino and the number of port.')
ERROR_VALUE_A = _('ERROR: Value must be a number from 0 to 1.')
ERROR_VALUE_S = _('ERROR: Value must be a number from 0 to 180.')
ERROR_VALUE_D = _('ERROR: Value must be either HIGH or LOW, 0 or 1')
ERROR_MODE = _('ERROR: The mode must be either INPUT, OUTPUT, PWM or SERVO.')
ERROR_VALUE_TYPE = _('ERROR: The value must be an integer.')
ERROR_PIN_TYPE = _('ERROR: The pin must be an integer.')
ERROR_PIN_CONFIGURED = _('ERROR: You must configure the mode for the pin.')

COLOR_NOTPRESENT = ["#A0A0A0","#808080"]
COLOR_PRESENT = ["#00FFFF","#00A0A0"]


class Arduino(Plugin):

    def __init__(self, parent):
        Plugin.__init__(self)
        self.tw = parent
        self._baud = 57600
        self.active_arduino = 0
        self._arduinos = []
        self._arduinos_it = []

    def setup(self):
        """ Setup is called once, when the Turtle Window is created. """

        palette = make_palette('arduino', COLOR_NOTPRESENT,
                             _('Palette of Arduino blocks'),
                             translation=_('arduino'))

        palette.add_block('arduinorefresh',
                     style='basic-style',
                     label=_('refresh Arduino'),
                     prim_name='arduinorefresh',
                     help_string=_('Search for connected Arduinos.'))
        self.tw.lc.def_prim('arduinorefresh', 0,
            Primitive(self.refresh))
        special_block_colors['arduinorefresh'] = COLOR_PRESENT[:]

        palette.add_block('arduinoselect',
                          style='basic-style-1arg',
                          default = 1,
                          label=_('Arduino'),
                          help_string=_('set current Arduino board'),
                          prim_name = 'arduinoselect')
        self.tw.lc.def_prim('arduinoselect', 1,
            Primitive(self.select, arg_descs=[ArgSlot(TYPE_NUMBER)]))

        palette.add_block('arduinocount',
                          style='box-style',
                          label=_('number of Arduinos'),
                          help_string=_('number of Arduino boards'),
                          prim_name = 'arduinocount')
        self.tw.lc.def_prim('arduinocount', 0,
            Primitive(self.count, TYPE_INT))

        palette.add_block('arduinoname',
                  style='number-style-1arg',
                  label=_('Arduino name'),
                  default=[1],
                  help_string=_('Get the name of an Arduino.'),
                  prim_name='arduinoname')
        self.tw.lc.def_prim('arduinoname', 1,
            Primitive(self.getName, TYPE_STRING, [ArgSlot(TYPE_NUMBER)]))
 
        palette.add_block('pinmode',
                  style='basic-style-2arg',
                  label=[_('pin mode'),_('pin'),_('mode')],
                  default=[1],
                  help_string=_('Select the pin function (INPUT, OUTPUT, PWM, SERVO).'),
                  prim_name='pinmode')
        self.tw.lc.def_prim('pinmode', 2,
            Primitive(self.pinMode, arg_descs=[ArgSlot(TYPE_NUMBER), ArgSlot(TYPE_STRING)]))

        palette.add_block('analogwrite',
                  style='basic-style-2arg',
                  label=[_('analog write'),_('pin'),_('value')],
                  default=[0, 1],
                  help_string=_('Write analog value in specified port.'),
                  prim_name='analogwrite')
        self.tw.lc.def_prim('analogwrite', 2,
            Primitive(self.analogWrite, arg_descs=[ArgSlot(TYPE_NUMBER), ArgSlot(TYPE_NUMBER)]))

        palette.add_block('analogread',
                  style='number-style-1arg',
                  label=[_('analog read')],
                  default=[0],
                  help_string=_('Read value from analog port. Value may be between 0 and 1.'),
                  prim_name='analogread')
        self.tw.lc.def_prim('analogread', 1,
            Primitive(self.analogRead, TYPE_FLOAT, arg_descs=[ArgSlot(TYPE_NUMBER)]))

        palette.add_block('digitalwrite',
                  style='basic-style-2arg',
                  label=[_('digital write'),_('pin'),_('value')],
                  default=[13, 1],
                  help_string=_('Write digital value to specified port.'),
                  prim_name='digitalwrite')
        self.tw.lc.def_prim('digitalwrite', 2,
            Primitive(self.digitalWrite, arg_descs=[ArgSlot(TYPE_NUMBER), ArgSlot(TYPE_NUMBER)]))

        palette.add_block('digitalread',
                  style='number-style-1arg',
                  label=[_('digital read')],
                  default=[13],
                  help_string=_('Read value from digital port.'),
                  prim_name='digitalread')
        self.tw.lc.def_prim('digitalread', 1,
            Primitive(self.digitalRead, TYPE_INT, arg_descs=[ArgSlot(TYPE_NUMBER)]))

        global CONSTANTS
        CONSTANTS['HIGH'] = 1
        palette.add_block('high',
                  style='box-style',
                  label=_('HIGH'),
                  help_string=_('Set HIGH value for digital port.'),
                  prim_name='high')
        self.tw.lc.def_prim('high', 0,
            Primitive(CONSTANTS.get, TYPE_INT, [ConstantArg('HIGH')]))

        CONSTANTS['INPUT'] = _('INPUT')
        palette.add_block('input',
                  style='box-style',
                  label=_('INPUT'),
                  help_string=_('Configure Arduino port for digital input.'),
                  prim_name='input')
        self.tw.lc.def_prim('input', 0,
            Primitive(CONSTANTS.get, TYPE_STRING, [ConstantArg('INPUT')]))

        CONSTANTS['SERVO'] = _('SERVO')
        palette.add_block('servo',
                  style='box-style',
                  label=_('SERVO'),
                  help_string=_('Configure Arduino port to drive a servo.'),
                  prim_name='servo')
        self.tw.lc.def_prim('servo', 0,
            Primitive(CONSTANTS.get, TYPE_STRING, [ConstantArg('SERVO')]))

        CONSTANTS['LOW'] = 0
        palette.add_block('low',
                  style='box-style',
                  label=_('LOW'),
                  help_string=_('Set LOW value for digital port.'),
                  prim_name='low')
        self.tw.lc.def_prim('low', 0,
            Primitive(CONSTANTS.get, TYPE_INT, [ConstantArg('LOW')]))

        CONSTANTS['OUTPUT'] = _('OUTPUT')
        palette.add_block('output',
                  style='box-style',
                  label=_('OUTPUT'),
                  help_string=_('Configure Arduino port for digital output.'),
                  prim_name='output')
        self.tw.lc.def_prim('output', 0,
            Primitive(CONSTANTS.get, TYPE_STRING, [ConstantArg('OUTPUT')]))

        CONSTANTS['PWM'] = _('PWM')
        palette.add_block('pwm',
                  style='box-style',
                  label=_('PWM'),
                  help_string=_('Configure Arduino port for PWM (pulse-width modulation).'),
                  prim_name='pwm')
        self.tw.lc.def_prim('pwm', 0,
            Primitive(CONSTANTS.get, TYPE_STRING, [ConstantArg('PWM')]))

    ############################### Turtle signals ############################

    def quit(self):
        for dev in self._arduinos:
            try:
                dev.exit()
            except:
                pass

    ###########################################################################

    def _check_init(self):
        n = len(self._arduinos)
        if (self.active_arduino > n) or (self.active_arduino < 0):
            raise logoerror(_('Not found Arduino %s') % (self.active_arduino + 1))

    def pinMode(self, pin, mode):
        self._check_init()
        try:
            pin = int(pin)
        except:
            raise logoerror(_('The pin must be an integer'))
        if (mode in MODE):
            try:
                a = self._arduinos[self.active_arduino]
                actual_mode = a.digital[pin]._get_mode()
                new_mode = MODE[mode]
                if not(actual_mode == new_mode):
                    a.digital[pin]._set_mode(new_mode)
            except:
                raise logoerror(ERROR)
        else:
            raise logoerror(ERROR_MODE)

    def analogWrite(self, pin, value):
        self._check_init()
        try:
            pin = int(pin)
        except:
            raise logoerror(ERROR_PIN_TYPE)
        try:
            tmp = float(value)
        except:
            raise logoerror(ERROR_VALUE_TYPE)
        try:
            a = self._arduinos[self.active_arduino]
            mode = a.digital[pin]._get_mode()
        except:
            raise logoerror(ERROR)
        if mode == MODE[_('PWM')]:
            min_value = 0.
            max_value = 1.
            error = ERROR_VALUE_A
        elif mode == MODE[_('SERVO')]:
            min_value = 0
            max_value = 180
            error = ERROR_VALUE_S
        else:
            raise logoerror(ERROR_PIN_CONFIGURED)
        if not((tmp < min_value) or (tmp > max_value)):
            try:
                a = self._arduinos[self.active_arduino]
                a.digital[pin].write(tmp)
            except:
                raise logoerror(ERROR)
        else:
            raise logoerror(error)

    def digitalWrite(self, pin, value):
        self._check_init()
        try:
            pin = int(pin)
        except:
            raise logoerror(ERROR_PIN_TYPE)
        try:
            value = int(value)
        except:
            raise logoerror(ERROR_VALUE_TYPE)
        try:
            a = self._arduinos[self.active_arduino]
            mode = a.digital[pin]._get_mode()
        except:
            raise logoerror(ERROR)
        if mode != MODE[_('OUTPUT')]:
            raise logoerror(ERROR_PIN_CONFIGURED)
        if (value < 0) or (value > 1):
            raise logoerror(ERROR_VALUE_D)
        else:
            try:
                a = self._arduinos[self.active_arduino]
                a.digital[pin].write(value)
            except:
                raise logoerror(ERROR)

    def analogRead(self, pin):
        self._check_init()
        try:
            pin = int(pin)
        except:
            raise logoerror(ERROR_PIN_TYPE)
        res = -1
        try:
            a = self._arduinos[self.active_arduino]
            a.analog[pin].enable_reporting()
            a.pass_time(0.05) # wait for the iterator to start receiving data
            res = a.analog[pin].read()
            a.digital[pin].disable_reporting()
        except:
            pass
        return res

    def digitalRead(self, pin):
        self._check_init()
        try:
            pin = int(pin)
        except:
            raise logoerror(ERROR_PIN_TYPE)
        try:
            a = self._arduinos[self.active_arduino]
            mode = a.digital[pin]._get_mode()
        except:
            raise logoerror(ERROR)
        if mode != MODE[_('INPUT')]:
            raise logoerror(ERROR_PIN_CONFIGURED)
        res = -1
        try:
            a = self._arduinos[self.active_arduino]
            a.digital[pin].enable_reporting()
            a.pass_time(0.05) # wait for the iterator to start receiving data
            if a.digital[pin].read() is None:
                # if the library returns None it is actually False  not being updated
                res = False
            else:
                res = a.digital[pin].read()
            a.digital[pin].disable_reporting()
        except:
            pass
        return res

    def select(self, i):
        n = len(self._arduinos)
        try:
            i = int(i)
        except:
            raise logoerror(_('The device must be an integer'))
        i = i - 1
        if (i < n) and (i >= 0):
            self.active_arduino = i
        else:
            raise logoerror(_('Not found Arduino %s') % (i + 1))

    def count(self):
        return len(self._arduinos)

    def getName(self, i):
        n = len(self._arduinos)
        try:
            i = int(i)
        except:
            raise logoerror(_('The device must be an integer'))
        i = i - 1
        if (i < n) and (i >= 0):
            a = self._arduinos[i]
            return a.name
        else:
            raise logoerror(_('Not found Arduino %s') % (i + 1))

    def change_color_blocks(self):
        if len(self._arduinos) > 0:
            arduino_present = True
        else:
            arduino_present = False
        index = palette_name_to_index('arduino')
        if index is not None:
            arduino_blocks = palette_blocks[index]
            for block in self.tw.block_list.list:
                if block.type in ['proto', 'block']:
                    if block.name in arduino_blocks:
                        if (arduino_present) or (block.name == 'arduinorefresh'):
                            special_block_colors[block.name] = COLOR_PRESENT[:]
                        else:
                            special_block_colors[block.name] = COLOR_NOTPRESENT[:]
                        block.refresh()
            self.tw.regenerate_palette(index)

    def refresh(self):
        #Close actual Arduinos
        for dev in self._arduinos:
            try:
                dev.exit()
            except:
                pass
        self._arduinos = []
        self._arduinos_it = []
        #Search for new Arduinos
        output_usb_parsed = []
        output_acm_parsed = []
        try:
            output_usb = subprocess.check_output('ls /dev/ | grep ttyUSB', shell=True)
            output_usb = output_usb.decode()
            output_usb_parsed = output_usb.split('\n')
        except:
            pass
        try:
            output_acm = subprocess.check_output('ls /dev/ | grep ttyACM', shell=True)
            output_acm = output_acm.decode()
            output_acm_parsed = output_acm.split('\n')
        except:
            pass
        output = output_usb_parsed
        output.extend(output_acm_parsed)
        for dev in output:
            if not(dev == ''):
                n = '/dev/%s' % dev
                try:
                    board = pyfirmata.Arduino(n, baudrate=self._baud)
                    it = pyfirmata.util.Iterator(board)
                    it.start()
                    self._arduinos.append(board)
                    self._arduinos_it.append(it)
                except:
                    raise logoerror(_('Error loading %s board') % n)
        self.change_color_blocks()

