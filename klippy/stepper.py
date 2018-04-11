# Printer stepper support
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import homing

# Tracking of shared stepper enable pins
class StepperEnablePin:
    def __init__(self, mcu_enable, enable_count=0):
        self.mcu_enable = mcu_enable
        self.enable_count = enable_count
    def set_enable(self, print_time, enable):
        if enable:
            if not self.enable_count:
                self.mcu_enable.set_digital(print_time, 1)
            self.enable_count += 1
        else:
            self.enable_count -= 1
            if not self.enable_count:
                self.mcu_enable.set_digital(print_time, 0)

def lookup_enable_pin(ppins, pin):
    if pin is None:
        return StepperEnablePin(None, 9999)
    pin_params = ppins.lookup_pin('digital_out', pin, 'stepper_enable')
    enable = pin_params.get('class')
    if enable is None:
        mcu_enable = pin_params['chip'].setup_pin(pin_params)
        mcu_enable.setup_max_duration(0.)
        pin_params['class'] = enable = StepperEnablePin(mcu_enable)
    return enable

# Code storing the definitions for a stepper motor
class PrinterStepper:
    def __init__(self, printer, config):
        self.name = config.get_name()
        if self.name.startswith('stepper_'):
            self.name = self.name[8:]
        self.need_motor_enable = True
        # Stepper definition
        ppins = printer.lookup_object('pins')
        self.mcu_stepper = ppins.setup_pin('stepper', config.get('step_pin'))
        dir_pin_params = ppins.lookup_pin('digital_out', config.get('dir_pin'))
        self.mcu_stepper.setup_dir_pin(dir_pin_params)
        self.step_dist = config.getfloat('step_distance', above=0.)
        self.mcu_stepper.setup_step_distance(self.step_dist)
        self.step = self.mcu_stepper.step
        self.step_const = self.mcu_stepper.step_const
        self.step_delta = self.mcu_stepper.step_delta
        self.enable = lookup_enable_pin(ppins, config.get('enable_pin', None))
    def _dist_to_time(self, dist, start_velocity, accel):
        # Calculate the time it takes to travel a distance with constant accel
        time_offset = start_velocity / accel
        return math.sqrt(2. * dist / accel + time_offset**2) - time_offset
    def set_max_jerk(self, max_halt_velocity, max_accel):
        # Calculate the firmware's maximum halt interval time
        last_step_time = self._dist_to_time(
            self.step_dist, max_halt_velocity, max_accel)
        second_last_step_time = self._dist_to_time(
            2. * self.step_dist, max_halt_velocity, max_accel)
        min_stop_interval = second_last_step_time - last_step_time
        self.mcu_stepper.setup_min_stop_interval(min_stop_interval)
    def set_position(self, pos):
        self.mcu_stepper.set_position(pos)
    def motor_enable(self, print_time, enable=0):
        if self.need_motor_enable != (not enable):
            self.enable.set_enable(print_time, enable)
        self.need_motor_enable = not enable

# Support for stepper controlled linear axis with an endstop
class PrinterHomingStepper(PrinterStepper):
    def __init__(self, printer, config, default_position=None):
        PrinterStepper.__init__(self, printer, config)
        # Endstop and its position
        ppins = printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', config.get('endstop_pin'))
        self.mcu_endstop.add_stepper(self.mcu_stepper)
        if default_position is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position)
        # Axis range
        self.position_min = config.getfloat('position_min', 0.)
        self.position_max = config.getfloat(
            'position_max', 0., above=self.position_min)
        # Homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean('homing_positive_dir', None)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'" % (
                        config.get_name(),))
        # Endstop stepper phase position tracking
        self.homing_stepper_phases = config.getint(
            'homing_stepper_phases', None, minval=0)
        endstop_accuracy = config.getfloat(
            'homing_endstop_accuracy', None, above=0.)
        self.homing_endstop_accuracy = self.homing_endstop_phase = None
        if self.homing_stepper_phases:
            self.homing_endstop_phase = config.getint(
                'homing_endstop_phase', None, minval=0
                , maxval=self.homing_stepper_phases-1)
            if (self.homing_endstop_phase is not None
                and config.getboolean('homing_endstop_align_zero', False)):
                # Adjust the endstop position so 0.0 is always at a full step
                micro_steps = self.homing_stepper_phases // 4
                phase_offset = (
                    ((self.homing_endstop_phase + micro_steps // 2) % micro_steps)
                    - micro_steps // 2) * self.step_dist
                full_step = micro_steps * self.step_dist
                es_pos = (int(self.position_endstop / full_step + .5) * full_step
                          + phase_offset)
                if es_pos != self.position_endstop:
                    logging.info("Changing %s endstop position to %.3f"
                                 " (from %.3f)", self.name, es_pos,
                                 self.position_endstop)
                    self.position_endstop = es_pos
            if endstop_accuracy is None:
                self.homing_endstop_accuracy = self.homing_stepper_phases//2 - 1
            elif self.homing_endstop_phase is not None:
                self.homing_endstop_accuracy = int(math.ceil(
                    endstop_accuracy * .5 / self.step_dist))
            else:
                self.homing_endstop_accuracy = int(math.ceil(
                    endstop_accuracy / self.step_dist))
            if self.homing_endstop_accuracy >= self.homing_stepper_phases // 2:
                logging.info("Endstop for %s is not accurate enough for stepper"
                             " phase adjustment", name)
                self.homing_stepper_phases = None
            if self.mcu_endstop.get_mcu().is_fileoutput():
                self.homing_endstop_accuracy = self.homing_stepper_phases
    def get_endstops(self):
        return [(self.mcu_endstop, self.name)]
    def get_homed_offset(self):
        if not self.homing_stepper_phases or self.need_motor_enable:
            return 0.
        pos = self.mcu_stepper.get_mcu_position()
        pos %= self.homing_stepper_phases
        if self.homing_endstop_phase is None:
            logging.info("Setting %s endstop phase to %d", self.name, pos)
            self.homing_endstop_phase = pos
            return 0.
        delta = (pos - self.homing_endstop_phase) % self.homing_stepper_phases
        if delta >= self.homing_stepper_phases - self.homing_endstop_accuracy:
            delta -= self.homing_stepper_phases
        elif delta > self.homing_endstop_accuracy:
            raise homing.EndstopError(
                "Endstop %s incorrect phase (got %d vs %d)" % (
                    self.name, pos, self.homing_endstop_phase))
        return delta * self.step_dist

# Wrapper for dual stepper motor support
class PrinterMultiStepper(PrinterHomingStepper):
    def __init__(self, printer, config):
        PrinterHomingStepper.__init__(self, printer, config)
        self.endstops = PrinterHomingStepper.get_endstops(self)
        self.extras = []
        self.all_step_const = [self.step_const]
        for i in range(1, 99):
            if not config.has_section(config.get_name() + str(i)):
                break
            extraconfig = config.getsection(config.get_name() + str(i))
            extra = PrinterStepper(printer, extraconfig)
            self.extras.append(extra)
            self.all_step_const.append(extra.step_const)
            extraendstop = extraconfig.get('endstop_pin', None)
            if extraendstop is not None:
                ppins = printer.lookup_object('pins')
                mcu_endstop = ppins.setup_pin('endstop', extraendstop)
                mcu_endstop.add_stepper(extra.mcu_stepper)
                self.endstops.append((mcu_endstop, extra.name))
            else:
                self.mcu_endstop.add_stepper(extra.mcu_stepper)
        self.step_const = self.step_multi_const
    def step_multi_const(self, print_time, start_pos, dist, start_v, accel):
        for step_const in self.all_step_const:
            step_const(print_time, start_pos, dist, start_v, accel)
    def set_max_jerk(self, max_halt_velocity, max_accel):
        PrinterHomingStepper.set_max_jerk(self, max_halt_velocity, max_accel)
        for extra in self.extras:
            extra.set_max_jerk(max_halt_velocity, max_accel)
    def set_position(self, pos):
        PrinterHomingStepper.set_position(self, pos)
        for extra in self.extras:
            extra.set_position(pos)
    def motor_enable(self, print_time, enable=0):
        PrinterHomingStepper.motor_enable(self, print_time, enable)
        for extra in self.extras:
            extra.motor_enable(print_time, enable)
    def get_endstops(self):
        return self.endstops

def LookupMultiHomingStepper(printer, config):
    if not config.has_section(config.get_name() + '1'):
        return PrinterHomingStepper(printer, config)
    return PrinterMultiStepper(printer, config)
