from tango import AttrWriteType, DispLevel, DevState
from tango.server import Device, attribute, command, device_property, GreenMode
from importlib.resources import files
import numpy as np
import asyncio


class EVR116Controller(Device):
    DAC_DUTY_CYCLE_VALUES = device_property(
        dtype=(float,),
        doc="duty cycle values",
        default_value=np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90]),
    )
    DAC_VOLTAGES_VALUES = device_property(
        dtype=(float,),
        doc="measured values for the given duty cycle values",
        default_value=np.array(
            [0, 1.4, 2.77, 4.11, 5.45, 6.76, 8.07, 9.36, 10.44, 10.61]
        ),
    )
    voltage = attribute(
        label="appl. voltage",
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        unit="V",
        format="%2.2f",
        max_value=10,
        min_value=0,
    )

    gasflow = attribute(
        label="gas flow",
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        unit="mbar l/s",
        format="%5.2f",
        min_value=0,
    )
    percentage = attribute(
        label="percent of flow",
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        unit="%",
        format="%2.2f",
        min_value=0,
        max_value=100,
    )

    readme = attribute(
        label="README",
        dtype=str,
        display_level=DispLevel.EXPERT,
        doc="""You need to do the following steps to calibrate the DAC:
    1) Disconnect the VakuPI cable from the valve.
    2) Connect the "+" of voltmeter measure device to the
    3d pinout of the cable (bottom middle)
    3) Connect the "-" of voltmeter to any
    neighbour pin 2nd or 4th (they both are GND)
    4) After you start the \"dac_calibration\" command
    the DAC's duty cycle will iterate in range of 0 to 90 (incl.)
    with step of 10. Each step will take 6 seconds. You have to
    remember the measured voltages
    5) When calibration is done set the measured values
    into DAC_VOLTAGES_VALUES property and restart the device.
    6) Ensure that the voltages do not overshoot the 10V. It can cause damages to the valve.""",
    )

    def read_readme(self):
        return "READ BEFORE ANY FURTHER STEPS"

    def read_voltage(self):
        return self._voltage

    def write_voltage(self, value):
        self.update_attrs(voltage=value)

    def read_gasflow(self):
        return self._gasflow

    def write_gasflow(self, value):
        self.update_attrs(gasflow=value)

    def read_percentage(self):
        return self._percentage

    def write_percentage(self, value):
        self.update_attrs(percentage=value)

    def update_attrs(self, voltage=None, gasflow=None, percentage=None):
        if voltage is not None and gasflow is None and percentage is None:
            self._voltage = voltage
            conv_gasflow = np.interp(
                voltage, self._curve_voltage, self._curve_pressure, left=0
            )
            self._gasflow = conv_gasflow
            self._percentage = conv_gasflow / self._max_gas_flow * 100
        elif gasflow is not None and voltage is None and percentage is None:
            self._gasflow = gasflow
            self._percentage = gasflow / self._max_gas_flow * 100
            conv_voltage = np.interp(
                gasflow, self._curve_pressure, self._curve_voltage, left=0, right=10
            )
            self._voltage = conv_voltage
        elif percentage is not None and voltage is None and gasflow is None:
            self._percentage = percentage
            conv_gasflow = percentage / 100 * self._max_gas_flow
            self._gasflow = conv_gasflow
            conv_voltage = np.interp(
                conv_gasflow,
                self._curve_pressure,
                self._curve_voltage,
                left=0,
                right=10,
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

    @command(display_level=DispLevel.EXPERT)
    def dac_calibration(self):
        self._command_loop.run_in_executor(None, self._dac_calibration)

    def _dac_calibration(self):
        import time

        self.set_state(DevState.MOVING)
        for duty_cycle in range(0, 100, 10):
            self._hardware_pwm.duty_cycle(duty_cycle)
            self.info_stream(f"Duty cycle: {duty_cycle}")
            time.sleep(6)
        self._hardware_pwm.duty_cycle(0)
        self.set_state(DevState.ON)

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
        duty_cycle, voltage = self.DAC_DUTY_CYCLE_VALUES, self.DAC_VOLTAGES_VALUES
        args = np.argsort(duty_cycle)
        self._duty_cycle, self._voltage_duty_cycle = duty_cycle[args], voltage[args]

    def init_device(self):
        Device.init_device(self)
        self.load_calibration_curves()
        self._voltage = 0
        self._gasflow = 0
        self._percentage = 0
        self._max_gas_flow = np.max(self._curve_pressure)
        self.init_valve()
        self._command_loop = asyncio.new_event_loop()
        self.set_state(DevState.ON)

    def delete_device(self):
        Device.delete_device(self)
        self.delete_valve()
