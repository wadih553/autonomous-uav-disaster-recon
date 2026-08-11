#!/usr/bin/env python3
"""
env_sensor_node.py
--------------------
Reads the two environmental sensors described in the FYP report (Ch. 3.2.3.3
and 3.2.3.4) and publishes them as ROS2 topics for the ground station
dashboard and for onboard fire-risk context.

  - SHT3X (I2C, addr 0x44): temperature (+-0.3 C) and humidity (+-2%)
  - MQ-135 (analog, via MCP3008 ADC channel 0, SPI): air quality / CO2 proxy

Wiring reference (block diagram / wiring_schematic.png):
  SHT3X  -> Raspberry Pi I2C (SDA/SCL)
  MQ-135 -> MCP3008 CH0 -> Raspberry Pi SPI (MOSI/MISO/SCLK/CE0)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import json

try:
    import smbus2
except ImportError:  # pragma: no cover
    smbus2 = None

try:
    import spidev
except ImportError:  # pragma: no cover
    spidev = None

SHT3X_ADDR = 0x44
SHT3X_MEAS_HIGHREP = [0x2C, 0x06]

MCP3008_CH_MQ135 = 0


class MCP3008:
    """Minimal MCP3008 SPI ADC driver (10-bit, single-ended)."""

    def __init__(self, bus=0, device=0, max_speed_hz=1350000):
        if spidev is None:
            self.spi = None
            return
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = max_speed_hz

    def read_channel(self, channel: int) -> int:
        if self.spi is None:
            return 0
        cmd = [1, (8 + channel) << 4, 0]
        resp = self.spi.xfer2(cmd)
        value = ((resp[1] & 3) << 8) + resp[2]
        return value  # 0-1023


class SHT3X:
    """Minimal SHT3X I2C driver (single-shot, high repeatability)."""

    def __init__(self, bus_num=1, address=SHT3X_ADDR):
        self.address = address
        self.bus = smbus2.SMBus(bus_num) if smbus2 else None

    def read(self):
        """Returns (temperature_C, humidity_pct) or (None, None) on failure."""
        if self.bus is None:
            return None, None
        try:
            self.bus.write_i2c_block_data(self.address, SHT3X_MEAS_HIGHREP[0],
                                           SHT3X_MEAS_HIGHREP[1:])
            time.sleep(0.015)
            data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
            temp_raw = data[0] << 8 | data[1]
            hum_raw = data[3] << 8 | data[4]
            temperature = -45 + 175 * (temp_raw / 65535.0)
            humidity = 100 * (hum_raw / 65535.0)
            return round(temperature, 2), round(humidity, 2)
        except Exception:
            return None, None


class EnvSensorNode(Node):
    def __init__(self):
        super().__init__('env_sensor_node')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('mq135_rl_ohm', 20000.0)   # load resistor
        self.declare_parameter('mq135_vref', 5.0)
        self.declare_parameter('mq135_r0', 76.63)  # sensor resistance in clean air, calibrate on-site

        self.temp_pub = self.create_publisher(Float32, 'drone/env/temperature', 10)
        self.humidity_pub = self.create_publisher(Float32, 'drone/env/humidity', 10)
        self.aqi_pub = self.create_publisher(Float32, 'drone/env/air_quality_index', 10)
        self.env_json_pub = self.create_publisher(String, 'drone/env/summary', 10)

        self.sht3x = SHT3X()
        self.adc = MCP3008()

        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._read_and_publish)
        self.get_logger().info('env_sensor_node started (SHT3X + MQ-135/MCP3008)')

    def _read_mq135_aqi(self) -> float:
        """Converts the raw MCP3008 reading into an approximate AQI-like index.

        This mirrors the calibration approach described in the report: the
        sensor is not a certified AQI instrument, it provides a relative
        CO2 / VOC proxy used for fire-risk situational awareness, not for
        regulatory reporting.
        """
        raw = self.adc.read_channel(MCP3008_CH_MQ135)
        if raw <= 0:
            return 0.0
        vref = self.get_parameter('mq135_vref').value
        rl = self.get_parameter('mq135_rl_ohm').value
        r0 = self.get_parameter('mq135_r0').value

        voltage = (raw / 1023.0) * vref
        if voltage <= 0.01:
            return 0.0
        rs = ((vref * rl) / voltage) - rl
        ratio = rs / r0 if r0 else 1.0
        # Empirical mapping consistent with typical MQ-135 CO2 curves;
        # calibrate mq135_r0 against a reference device before field use.
        aqi_like = max(0.0, 116.6020682 * (ratio ** -2.769034857) - 2.98)
        return round(aqi_like, 1)

    def _read_and_publish(self):
        temperature, humidity = self.sht3x.read()
        aqi = self._read_mq135_aqi()

        if temperature is not None:
            t_msg = Float32(); t_msg.data = temperature
            self.temp_pub.publish(t_msg)
        if humidity is not None:
            h_msg = Float32(); h_msg.data = humidity
            self.humidity_pub.publish(h_msg)

        a_msg = Float32(); a_msg.data = aqi
        self.aqi_pub.publish(a_msg)

        summary = {
            'temperature_c': temperature,
            'humidity_pct': humidity,
            'air_quality_index': aqi,
            'timestamp': self.get_clock().now().to_msg().sec,
        }
        s_msg = String(); s_msg.data = json.dumps(summary)
        self.env_json_pub.publish(s_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnvSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
