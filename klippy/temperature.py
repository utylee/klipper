# Base support for temperature sensors
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging


######################################################################
# ADC based sensors
######################################################################

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.300
KELVIN_TO_CELCIUS = -273.15

class ADCTemperatureBase:
    def __init__(self, config, params):
        self.callback = None
        printer = config.get_printer()
        ppins = printer.lookup_object('pins')
        min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELCIUS)
        max_temp = config.getfloat('max_temp', above=min_temp)
        mcu_adc = ppins.setup_pin('adc', config.get('sensor_pin'))
        adc_range = [self.calc_adc(min_temp), self.calc_adc(max_temp)]
        mcu_adc.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT,
                                  minval=min(adc_range), maxval=max(adc_range))
        mcu_adc.setup_adc_callback(REPORT_TIME, self.adc_callback)
    def setup_callback(self, callback):
        self.callback = callback
    def get_report_delta(self):
        return REPORT_TIME + SAMPLE_TIME * SAMPLE_COUNT

# Thermistor calibrated with three temp measurements
class Thermistor(ADCTemperatureBase):
    def __init__(self, config, params):
        self.pullup = config.getfloat('pullup_resistor', 4700., above=0.)
        # Calculate Steinhart-Hart coefficents from temp measurements
        inv_t1 = 1. / (params['t1'] - KELVIN_TO_CELCIUS)
        inv_t2 = 1. / (params['t2'] - KELVIN_TO_CELCIUS)
        inv_t3 = 1. / (params['t3'] - KELVIN_TO_CELCIUS)
        ln_r1 = math.log(params['r1'])
        ln_r2 = math.log(params['r2'])
        ln_r3 = math.log(params['r3'])
        ln3_r1, ln3_r2, ln3_r3 = ln_r1**3, ln_r2**3, ln_r3**3

        inv_t12, inv_t13 = inv_t1 - inv_t2, inv_t1 - inv_t3
        ln_r12, ln_r13 = ln_r1 - ln_r2, ln_r1 - ln_r3
        ln3_r12, ln3_r13 = ln3_r1 - ln3_r2, ln3_r1 - ln3_r3

        self.c3 = ((inv_t12 - inv_t13 * ln_r12 / ln_r13)
                   / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13))
        self.c2 = (inv_t12 - self.c3 * ln3_r12) / ln_r12
        self.c1 = inv_t1 - self.c2 * ln_r1 - self.c3 * ln3_r1
        ADCTemperatureBase.__init__(self, config, params)
    def adc_callback(self, read_time, adc):
        adc = max(.00001, min(.99999, adc))
        r = self.pullup * adc / (1.0 - adc)
        ln_r = math.log(r)
        inv_t = self.c1 + self.c2 * ln_r + self.c3 * ln_r**3
        self.callback(read_time, 1.0/inv_t + KELVIN_TO_CELCIUS)
    def calc_adc(self, temp):
        inv_t = 1. / (temp - KELVIN_TO_CELCIUS)
        if self.c3:
            y = (self.c1 - inv_t) / (2. * self.c3)
            x = math.sqrt((self.c2 / (3. * self.c3))**3 + y**2)
            ln_r = math.pow(x - y, 1./3.) - math.pow(x + y, 1./3.)
        else:
            ln_r = (inv_t - self.c1) / self.c2
        r = math.exp(ln_r)
        return r / (self.pullup + r)

# Thermistor calibrated from one temp measurement and its beta
class ThermistorBeta(Thermistor):
    def __init__(self, config, params):
        self.pullup = config.getfloat('pullup_resistor', 4700., above=0.)
        # Calculate Steinhart-Hart coefficents from beta
        inv_t1 = 1. / (params['t1'] - KELVIN_TO_CELCIUS)
        ln_r1 = math.log(params['r1'])
        self.c3 = 0.
        self.c2 = 1. / params['beta']
        self.c1 = inv_t1 - self.c2 * ln_r1
        ADCTemperatureBase.__init__(self, config, params)

# Linear style conversion chips calibrated with two temp measurements
class Linear(ADCTemperatureBase):
    def __init__(self, config, params):
        adc_voltage = config.getfloat('adc_voltage', 5., above=0.)
        slope = (params['t2'] - params['t1']) / (params['v2'] - params['v1'])
        self.gain = adc_voltage * slope
        self.offset = params['t1'] - params['v1'] * slope
        ADCTemperatureBase.__init__(self, config, params)
    def adc_callback(self, read_time, adc):
        self.callback(read_time, adc * self.gain + self.offset)
    def calc_adc(self, temp):
        return (temp - self.offset) / self.gain


######################################################################
# Sensor registration
######################################################################

class PrinterTemperatureSensors:
    def __init__(self, printer):
        self.printer = printer
        self.sensors = {}
    def add_sensor(self, name, params):
        self.sensors[name] = params
    def create_sensor(self, name, config):
        if name not in self.sensors:
            raise self.printer.config_error("Unknown temperature sensor '%s'" % (
                name,))
        sensor = self.sensors[name]
        return sensor['class'](config, sensor)

# Default sensors available
DefaultSensors = {
    "EPCOS 100K B57560G104F": {
        'class': Thermistor, 't1': 25., 'r1': 100000.,
        't2': 150., 'r2': 1641.9, 't3': 250., 'r3': 226.15 },
    "ATC Semitec 104GT-2": {
        'class': Thermistor, 't1': 20., 'r1': 126800.,
        't2': 150., 'r2': 1360., 't3': 300., 'r3': 80.65 },
    "NTC 100K beta 3950": {
        'class': ThermistorBeta, 't1': 25., 'r1': 100000., 'beta': 3950. },
    "AD595": { 'class': Linear, 't1': 25., 'v1': .25, 't2': 300., 'v2': 3.022 },
}

def add_printer_objects(printer, config):
    pts = PrinterTemperatureSensors(printer)
    printer.add_object('temperature_sensors', pts)
    for name, params in DefaultSensors.items():
        pts.add_sensor(name, params)
