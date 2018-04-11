# Bed tilt compensation
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import probe, mathutil

class BedTilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.x_adjust = config.getfloat('x_adjust', 0.)
        self.y_adjust = config.getfloat('y_adjust', 0.)
        self.z_adjust = 0.
        if config.get('points', None) is not None:
            BedTiltCalibrate(config, self)
        self.toolhead = None
        gcode = self.printer.lookup_object('gcode')
        gcode.set_move_transform(self)
    def printer_state(self, state):
        if state == 'connect':
            self.toolhead = self.printer.lookup_object('toolhead')
    def get_position(self):
        x, y, z, e = self.toolhead.get_position()
        return [x, y, z - x*self.x_adjust - y*self.y_adjust - self.z_adjust, e]
    def move(self, newpos, speed):
        x, y, z, e = newpos
        self.toolhead.move([x, y, z + x*self.x_adjust + y*self.y_adjust
                            + self.z_adjust, e], speed)

# Helper script to calibrate the bed tilt
class BedTiltCalibrate:
    def __init__(self, config, bedtilt):
        self.bedtilt = bedtilt
        self.printer = config.get_printer()
        points = config.get('points').split('\n')
        try:
            points = [line.split(',', 1) for line in points if line.strip()]
            self.points = [(float(p[0].strip()), float(p[1].strip()))
                           for p in points]
        except:
            raise config.error("Unable to parse bed tilt points")
        if len(self.points) < 3:
            raise config.error("Need at least 3 points for bed_tilt_calibrate")
        self.speed = config.getfloat('speed', 50., above=0.)
        self.horizontal_move_z = config.getfloat('horizontal_move_z', 5.)
        self.z_position_endstop = None
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position_endstop = zconfig.getfloat('position_endstop', None)
        self.manual_probe = config.getboolean('manual_probe', None)
        if self.manual_probe is None:
            self.manual_probe = not config.has_section('probe')
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'BED_TILT_CALIBRATE', self.cmd_BED_TILT_CALIBRATE,
            desc=self.cmd_BED_TILT_CALIBRATE_help)
    cmd_BED_TILT_CALIBRATE_help = "Bed tilt calibration script"
    def cmd_BED_TILT_CALIBRATE(self, params):
        self.gcode.run_script("G28")
        probe.ProbePointsHelper(
            self.printer, self.points, self.horizontal_move_z,
            self.speed, self.manual_probe, self)
    def get_position(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        return kin.get_position()
    def finalize(self, z_offset, positions):
        logging.info("Calculating bed_tilt with: %s", positions)
        params = { 'x_adjust': self.bedtilt.x_adjust,
                   'y_adjust': self.bedtilt.y_adjust,
                   'z_adjust': z_offset }
        logging.info("Initial bed_tilt parameters: %s", params)
        def adjusted_height(pos, params):
            x, y, z = pos
            return (z - x*params['x_adjust'] - y*params['y_adjust']
                    - params['z_adjust'])
        def errorfunc(params):
            total_error = 0.
            for pos in positions:
                total_error += adjusted_height(pos, params)**2
            return total_error
        new_params = mathutil.coordinate_descent(
            params.keys(), params, errorfunc)
        logging.info("Calculated bed_tilt parameters: %s", new_params)
        for pos in positions:
            logging.info("orig: %s new: %s", adjusted_height(pos, params),
                         adjusted_height(pos, new_params))
        # Update current bed_tilt calculations
        bed_tilt = self.printer.lookup_object('bed_tilt')
        bed_tilt.x_adjust = new_params['x_adjust']
        bed_tilt.y_adjust = new_params['y_adjust']
        z_diff = new_params['z_adjust'] - z_offset
        bed_tilt.z_adjust = z_diff
        self.gcode.reset_last_position()
        # Report results back to user
        if self.z_position_endstop is not None:
            # Cartesian style robot
            z_extra = ""
            probe = self.printer.lookup_object('probe', None)
            if probe is not None:
                last_home_position = probe.last_home_position()
                if last_home_position is not None:
                    # Using z_virtual_endstop
                    home_x, home_y = last_home_position[:2]
                    z_diff -= home_x * new_params['x_adjust']
                    z_diff -= home_y * new_params['y_adjust']
                    z_extra = " (when Z homing at %.3f,%.3f)" % (home_x, home_y)
            z_adjust = "stepper_z position_endstop: %.6f%s\n" % (
                self.z_position_endstop - z_diff, z_extra)
        else:
            # Delta (or other) style robot
            z_adjust = "Add %.6f to endstop position\n" % (-z_diff,)
        msg = "%sx_adjust: %.6f y_adjust: %.6f" % (
            z_adjust, new_params['x_adjust'], new_params['y_adjust'])
        self.printer.set_rollover_info("bed_tilt", "bed_tilt: %s" % (msg,))
        self.gcode.respond_info(
            "%s\nThe above parameters have been applied to the current\n"
            "session. Update the printer config file with the above to\n"
            "use these settings in future sessions." % (msg,))

def load_config(config):
    return BedTilt(config)
