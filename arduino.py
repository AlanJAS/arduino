#!/usr/bin/env python
# Copyright (c) 2012, Alan Aguiar <alanjas@hotmail.com>
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
import commands

from gettext import gettext as _
from plugins.plugin import Plugin

from TurtleArt.tapalette import make_palette
from TurtleArt.tapalette import palette_name_to_index
from TurtleArt.tapalette import palette_blocks
from TurtleArt.tapalette import special_block_colors
from TurtleArt.talogo import primitive_dictionary, logoerror
from TurtleArt.tautils import debug_output

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
        self._arduinos_dev = []
        self._arduinos_it = []

    def setup(self):
        """ Setup is called once, when the Turtle Window is created. """
        debug_output('creating %s palette' % _('arduino'), self.tw.running_sugar)
        palette = make_palette('arduino', COLOR_NOTPRESENT, _('Palette of Arduino blocks'))

        primitive_dictionary['arduinorefresh'] = self._prim_arduinorefresh
        palette.add_block('arduinorefresh',
                     style='basic-style',
                     label=_('refresh Arduino'),
                     prim_name='arduinorefresh',
                     help_string=_('Search for connected Arduinos.'))
        self.tw.lc.def_prim('arduinorefresh', 0, lambda self : primitive_dictionary['arduinorefresh']())
        special_block_colors['arduinorefresh'] = COLOR_PRESENT[:]

        primitive_dictionary['arduinoselect'] = self._prim_arduinoselect
        palette.add_block('arduinoselect',
                          style='basic-style-1arg',
                          default = 1,
                          label=_('Arduino'),
                          help_string=_('set current Arduino board'),
                          prim_name = 'arduinoselect')
        self.tw.lc.def_prim('arduinoselect', 1, lambda self, n : primitive_dictionary['arduinoselect'](n))

        primitive_dictionary['arduinocount'] = self._prim_arduinocount
        palette.add_block('arduinocount',
                          style='box-style',
                          label=_('number of Arduinos'),
                          help_string=_('number of Arduino boards'),
                          prim_name = 'arduinocount')
        self.tw.lc.def_prim('arduinocount', 0,
            lambda self:
            primitive_dictionary['arduinocount']())

        primitive_dictionary['arduinoname'] = self._prim_arduinoname
        palette.add_block('arduinoname',
                  style='number-style-1arg',
                  label=_('Arduino name'),
                  default=[1],
                  help_string=_('Get the name of an Arduino.'),
                  prim_name='arduinoname')
        self.tw.lc.def_prim('arduinoname', 1,
            lambda self, x:
            primitive_dictionary['arduinoname'](x))

        primitive_dictionary['pinmode'] = self._prim_pin_mode
        palette.add_block('pinmode',
                  style='basic-style-2arg',
                  label=[_('pin mode'),_('pin'),_('mode')],
                  default=[1],
                  help_string=_('Select the pin function (INPUT, OUTPUT, PWM, SERVO).'),
                  prim_name='pinmode')
        self.tw.lc.def_prim('pinmode', 2, lambda self, x, y : primitive_dictionary['pinmode'](x, y))

        primitive_dictionary['analogwrite'] = self._prim_analog_write
        palette.add_block('analogwrite',
                  style='basic-style-2arg',
                  label=[_('analog write'),_('pin'),_('value')],
                  default=[0, 255],
                  help_string=_('Write analog value in specified port.'),
                  prim_name='analogwrite')
        self.tw.lc.def_prim('analogwrite', 2,
            lambda self, x, y:
            primitive_dictionary['analogwrite'](x, y))

        primitive_dictionary['analogread'] = self._prim_analog_read
        palette.add_block('analogread',
                  style='number-style-1arg',
                  label=[_('analog read')],
                  default=[0],
                  help_string=_('Read value from analog port. Value may be between 0 and 1.'),
                  prim_name='analogread')
        self.tw.lc.def_prim('analogread', 1,
            lambda self, x:
            primitive_dictionary['analogread'](x))

        primitive_dictionary['digitalwrite'] = self._prim_digital_write
        palette.add_block('digitalwrite',
                  style='basic-style-2arg',
                  label=[_('digital write'),_('pin'),_('value')],
                  default=[13, 1],
                  help_string=_('Write digital value to specified port.'),
                  prim_name='digitalwrite')
        self.tw.lc.def_prim('digitalwrite', 2,
            lambda self, x, y:
            primitive_dictionary['digitalwrite'](x, y))

        primitive_dictionary['digitalread'] = self._prim_digital_read
        palette.add_block('digitalread',
                  style='number-style-1arg',
                  label=[_('digital read')],
                  default=[13],
                  help_string=_('Read value from digital port.'),
                  prim_name='digitalread')
        self.tw.lc.def_prim('digitalread', 1,
            lambda self, x:
            primitive_dictionary['digitalread'](x))

        primitive_dictionary['high'] = self._prim_high
        palette.add_block('high',
                  style='box-style',
                  label=_('HIGH'),
                  help_string=_('Set HIGH value for digital port.'),
                  prim_name='high')
        self.tw.lc.def_prim('high', 0,
            lambda self: primitive_dictionary['high']())

        primitive_dictionary['input'] = self._prim_input
        palette.add_block('input',
                  style='box-style',
                  label=_('INPUT'),
                  help_string=_('Configure Arduino port for digital input.'),
                  prim_name='input')
        self.tw.lc.def_prim('input', 0,
            lambda self: primitive_dictionary['input']())

        primitive_dictionary['servo'] = self._prim_servo
        palette.add_block('servo',
                  style='box-style',
                  label=_('SERVO'),
                  help_string=_('Configure Arduino port to drive a servo.'),
                  prim_name='servo')
        self.tw.lc.def_prim('servo', 0,
            lambda self: primitive_dictionary['servo']())

        primitive_dictionary['low'] = self._prim_low
        palette.add_block('low',
                  style='box-style',
                  label=_('LOW'),
                  help_string=_('Set LOW value for digital port.'),
                  prim_name='low')
        self.tw.lc.def_prim('low', 0,
            lambda self: primitive_dictionary['low']())

        primitive_dictionary['output'] = self._prim_output
        palette.add_block('output',
                  style='box-style',
                  label=_('OUTPUT'),
                  help_string=_('Configure Arduino port for digital output.'),
                  prim_name='output')
        self.tw.lc.def_prim('output', 0,
            lambda self: primitive_dictionary['output']())

        primitive_dictionary['pwm'] = self._prim_pwm
        palette.add_block('pwm',
                  style='box-style',
                  label=_('PWM'),
                  help_string=_('Configure Arduino port for PWM (pulse-width modulation).'),
                  prim_name='pwm')
        self.tw.lc.def_prim('pwm', 0,
            lambda self: primitive_dictionary['pwm']())

    ############################### Turtle signals ############################

    def start(self):
        pass

    def quit(self):
        for dev in self._arduinos:
            try:
                dev.exit()
            except:
                pass

    def stop(self):
        pass

    ###########################################################################

    def _check_init(self):
        n = len(self._arduinos)
        if (self.active_arduino > n) or (self.active_arduino < 0):
            raise logoerror(_('Not found Arduino %s') % (self.active_arduino + 1))

    def _prim_pin_mode(self, pin, mode):
        self._check_init()
        try:
            pin = int(pin)
        except:
            raise logoerror(_('The pin must be an integer'))
        if (mode in MODE):
            try:
                a = self._arduinos[self.active_arduino]
                a.digital[pin]._set_mode(MODE[mode])
            except:
                raise logoerror(ERROR)
        else:
            raise logoerror(ERROR_MODE)

    def _prim_analog_write(self, pin, value):
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

    def _prim_digital_write(self, pin, value):
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

    def _prim_analog_read(self, pin):
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

    def _prim_digital_read(self, pin):
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

    def _prim_high(self):
        return 1

    def _prim_low(self):
        return 0

    def _prim_input(self):
        return _('INPUT')

    def _prim_output(self):
        return _('OUTPUT')

    def _prim_pwm(self):
        return _('PWM')

    def _prim_servo(self):
        return _('SERVO')

    def _prim_arduinoselect(self, i):
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

    def _prim_arduinocount(self):
        return len(self._arduinos)

    def _prim_arduinoname(self, i):
        n = len(self._arduinos)
        try:
            i = int(i)
        except:
            raise logoerror(_('The device must be an integer'))
        i = i - 1
        if (i < n) and (i >= 0):
            return self._arduinos_dev[i]
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

    def _prim_arduinorefresh(self):

        #Close actual Arduinos
        for dev in self._arduinos:
            try:
                dev.exit()
            except:
                pass
        self._arduinos = []
        self._arduinos_dev = []
        self._arduinos_it = []

        #Search for new Arduinos
        status,output_usb = commands.getstatusoutput("ls /dev/ | grep ttyUSB")
        output_usb_parsed = output_usb.split('\n')
        status,output_acm = commands.getstatusoutput("ls /dev/ | grep ttyACM")
        output_acm_parsed = output_acm.split('\n')
        output = output_usb_parsed
        output.extend(output_acm_parsed)

        for dev in output:
            if not(dev == ''):
                n = '/dev/%s' % dev
                try:
                    board = pyfirmata.Arduino(n, baudrate=self._baud)
                    self._arduinos.append(board)
                    self._arduinos_dev.append(n)
                    self._arduinos_it.append(pyfirmata.util.Iterator(board))
                    self._arduinos_it[len(self._arduinos)-1].start()
                except:
                    raise logoerror(_('Error loading %s board') % n)

        self.change_color_blocks()
