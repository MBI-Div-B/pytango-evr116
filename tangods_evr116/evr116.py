from tango import AttrWriteType, DispLevel, DevState
from tango.server import Device, attribute, command, device_property
from importlib.resources import files
import numpy as np


class EVR116Controller(Device):
    voltage = attribute(
        label="appl. voltage",
        dtype=float,
        unit="V",
        format="%.2f",
        max_value=10,
        min_value=0,
    )

    gasflow = attribute(
        label="appl. voltage",
        dtype=float,
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
        elif gasflow is not None and gasflow is None:
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

    def init_valve(self):
        from rpi_hardware_pwm import HardwarePWM

        self._hardware_pwm = HardwarePWM(pwm_channel=0, hz=1_000)
        self._hardware_pwm.start(0)

    def delete_valve(self):
        self._hardware_pwm.duty_cycle(0)
        self._hardware_pwm.stop()

    def send_to_valve(self):
        value_to_set = min(max((self._voltage / 10) * 100, 0), 100)
        self._hardware_pwm.change_duty_cycle(value_to_set)

    def load_calibration_curve(self):
        table_path = files("tangods_evr116.calibration").joinpath(
            "pressure_voltage_curve.csv"
        )

        voltage, pressure = np.loadtxt(table_path, delimiter=";", unpack=True)
        args = np.argsort(voltage)
        self._curve_voltage, self._curve_pressure = voltage[args], pressure[args]

    def init_device(self):
        Device.init_device(self)
        self._voltage = 0
        self._gasflow = 0
        self.init_valve()
        self.set_state(DevState.ON)

    def delete_device(self):
        Device.delete_device(self)
        self.delete_valve()
