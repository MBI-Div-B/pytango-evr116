from tango import AttrWriteType, DispLevel, DevState
from tango.server import Device, attribute, command, device_property
from importlib.resources import files
import numpy as np


class EVR116Controller(Device):
    voltage = attribute(
        label="appl. voltage",
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        unit="V",
        format="%.2f",
        max_value=10,
        min_value=0,
    )

    gasflow = attribute(
        label="gas flow",
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        unit="mbar l/s",
        format="%.2f",
        min_value=0,
    )

    def read_voltage(self):
        return self._voltage

    def write_voltage(self, value):
        self.update_attrs(voltage=value)

    def read_gasflow(self):
        return self._gasflow

    def write_gasflow(self, value):
        self.update_attrs(gasflow=value)

    def update_attrs(self, voltage=None, gasflow=None):
        if voltage is not None and gasflow is None:
            self._voltage = voltage
            conv_gasflow = np.interp(
                voltage, self._curve_voltage, self._curve_pressure, left=0
            )
            self._gasflow = conv_gasflow
        elif gasflow is not None and voltage is None:
            self._gasflow = gasflow
            conv_voltage = np.interp(
                gasflow, self._curve_pressure, self._curve_voltage, left=0, right=10
            )
            self._voltage = conv_voltage
        else:
            raise Exception()

    @command
    def apply(self):
        self.send_to_valve()

    @command(dtype_in=str, doc_in="type any character to confirm")
    def open_completely(self, confirm):
        if confirm:
            self.write_voltage(10)
            self.apply()

    @command(dtype_in=str, doc_in="type any character to confirm")
    def close_completely(self, confirm):
        if confirm:
            self.write_voltage(0)
            self.apply()

    def init_valve(self):
        from rpi_hardware_pwm import HardwarePWM

        self._hardware_pwm = HardwarePWM(pwm_channel=0, hz=1_000)
        self._hardware_pwm.start(0)

    def delete_valve(self):
        self._hardware_pwm.duty_cycle(0)
        self._hardware_pwm.stop()

    def send_to_valve(self):
        # 10.60V is vacupi output in 100 duty cycle
        value_to_set = np.interp(
            self._voltage,
            self._voltage_duty_cycle,
            self._duty_cycle,
        )
        self._hardware_pwm.change_duty_cycle(value_to_set)

    def load_calibration_curves(self):
        # load pfeiffer calibration curve for dependency of gas flow as function of applied voltage
        pfeiffer_table_path = files("tangods_evr116.calibration").joinpath(
            "pressure_voltage_curve.csv"
        )
        voltage, pressure = np.loadtxt(pfeiffer_table_path, delimiter=";", unpack=True)
        # sort is necessary since the order is relevant for interpolation
        args = np.argsort(voltage)
        self._curve_voltage, self._curve_pressure = voltage[args], pressure[args]

        # load DAC calibration curve for converting PWM duty cycle parameter of PWM
        # since the DAC is not linearly mapping 0..100 duty cycle to 0..10.61 Volts
        voltage_curve_path = files("tangods_evr116.calibration").joinpath(
            "duty_cycle_voltage_curve.csv"
        )
        cycle, voltage = np.loadtxt(voltage_curve_path, delimiter=";", unpack=True)
        args = np.argsort(cycle)
        self._duty_cycle, self._voltage_duty_cycle = cycle[args], voltage[args]

    def init_device(self):
        Device.init_device(self)
        self.load_calibration_curves()
        self._voltage = 0
        self._gasflow = 0
        self.init_valve()
        self.set_state(DevState.ON)

    def delete_device(self):
        Device.delete_device(self)
        self.delete_valve()
