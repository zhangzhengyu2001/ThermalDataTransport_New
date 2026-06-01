"""
digital_power.py

数字电源控制类（Modbus RTU，基于 pyserial）。

协议约定（均以十六进制描述，发送与响应均不含 CRC 校验）：
- 读操作：地址(1) + 功能码03(1) + 寄存器地址(2) + 数量(2)
	返回：地址(1) + 功能码(1) + 寄存器地址(2) + 数据(2)
- 写操作：地址(1) + 功能码06(1) + 寄存器地址(2) + 设置值(2)
	返回：地址(1) + 功能码(1) + 寄存器地址(2) + 设置值(2)

电压与寄存器值之间的换算：
- 若电源最大量程为 y（单位：V），期望输出电压为 x（单位：V），
	则设置的寄存器值 S = round_half_up( x / (y/5) * 1000 )。
	例如 y=36V，x=30V：S = round(30/(36/5)*1000) = round(4166.66) = 4167，十六进制为 0x1047。
- 从寄存器值反算电压：x = (S/1000) * (y/5)。

注意：
- 本实现默认寄存器地址为 0x0001，读数量默认 0x0001，可通过参数覆盖。
"""

from __future__ import annotations

import time
from typing import List, Optional

import serial


def _round_half_up(x: float) -> int:
	"""四舍五入到最接近的整数（.5 向上）。

	Python 内置 round 使用 bankers rounding，这里改为常见“四舍五入”。
	"""
	import math
	return int(math.floor(x + 0.5))


class ModbusError(RuntimeError):
	pass


class DigitalPowerController:
	"""
	数字电源控制类，通过串口使用 Modbus RTU 协议进行读取/设置。

	参数：
	- port: 串口号，例如 'COM3'
	- addr: 从站地址，默认 0x01
	- baudrate: 波特率，默认 9600
	- timeout: 串口读超时（秒），默认 0.2
	- bytesize, parity, stopbits: 串口参数（见 pyserial）

	关键方法：
	- read_register(reg=0x0001, count=1) -> List[int]
	- write_register(reg=0x0001, value=int) -> None
	- set_voltage(voltage_v: float, max_range_v: float, reg=0x0001) -> int
	- read_voltage(max_range_v: float, reg=0x0001) -> float
	"""

	def __init__(
		self,
		port: str,
		addr: int = 0x01,
		baudrate: int = 9600,
		timeout: float = 0.2,
		bytesize: int = 8,
		parity: str = 'N',
		stopbits: int = 1,
	) -> None:
		self.addr = addr & 0xFF
		self.ser = serial.Serial(
			port=port,
			baudrate=baudrate,
			bytesize=bytesize,
			parity=parity,
			stopbits=stopbits,
			timeout=timeout,
		)

	# -------------------- 内部工具 --------------------
	def _read_exact(self, nbytes: int) -> bytes:
		"""读取精确 nbytes 字节，受串口 timeout 限制。"""
		buf = bytearray()
		while len(buf) < nbytes:
			chunk = self.ser.read(nbytes - len(buf))
			if not chunk:
				break
			buf.extend(chunk)
		return bytes(buf)

	def _tx_rx(self, payload_wo_crc: bytes, expect_len: int) -> bytes:
		# 不添加 CRC，直接发送原始负载
		self.ser.write(payload_wo_crc)
		resp = self._read_exact(expect_len)
		if len(resp) != expect_len:
			raise ModbusError(f"串口读取长度不足，期望 {expect_len} 字节，实际 {len(resp)} 字节")
		return resp

	# -------------------- 基础读写 --------------------
	def read_register(self, reg: int = 0x0001, count: int = 0x0001) -> List[int]:
		"""读取保持寄存器（功能码 0x03）。

		返回寄存器值列表，按寄存器顺序，每个为 0~65535 的整数。
		"""
		reg &= 0xFFFF
		count &= 0xFFFF
		payload = bytes([
			self.addr,
			0x03,
			(reg >> 8) & 0xFF,
			reg & 0xFF,
			(count >> 8) & 0xFF,
			count & 0xFF,
		])
		# 响应长度（无CRC）：addr(1)+func(1)+len(1)+data(2*count)
		expect_len = 4 + 2 * count
		resp = self._tx_rx(payload, expect_len)
		if resp[0] != self.addr or resp[1] != 0x03:
			raise ModbusError("响应地址或功能码不匹配")
		# byte_count = resp[2]
		# if byte_count != 2 * count:
		# 	print("预期长度:{}, 实际返回长度:{}".format(2*count,byte_count))
		# 	raise ModbusError("响应数据长度与期望不一致,发送:{}，实际返回：{}".format(payload,resp))
		data = resp[4:6]
		values: List[int] = []
		for i in range(0, len(data), 2):
			values.append((data[i] << 8) | data[i + 1])
		return values

	def write_register(self, reg: int = 0x0001, value: int = 0) -> None:
		"""写单个保持寄存器（功能码 0x06）。"""
		reg &= 0xFFFF
		value &= 0xFFFF
		payload = bytes([
			self.addr,
			0x06,
			(reg >> 8) & 0xFF,
			reg & 0xFF,
			(value >> 8) & 0xFF,
			value & 0xFF,
		])
		# 响应长度固定（无CRC）：addr(1)+func(1)+reg(2)+value(2) = 6
		resp = self._tx_rx(payload, 6)
		if resp[:6] != payload:
			raise ModbusError("写入回显与发送不一致")

	# -------------------- 电压与寄存器换算 --------------------
	@staticmethod
	def voltage_to_register(voltage_v: float, max_range_v: float) -> int:
		"""电压 -> 寄存器值（四舍五入，.5 向上）。

		S = round_half_up( voltage / (max_range_v / 5) * 1000 )
		并限制在 [0, 5000] 的合理范围内。
		"""
		if max_range_v <= 0:
			raise ValueError("max_range_v 必须为正")
		scale = voltage_v / (max_range_v / 5.0) * 1000.0
		s = _round_half_up(scale)
		# 对于 0~满量程，S 合理范围通常在 0~5000 之间
		if s < 0:
			s = 0
		if s > 5000:
			s = 5000
		return int(s)

	@staticmethod
	def register_to_voltage(reg_value: int, max_range_v: float) -> float:
		"""寄存器值 -> 电压。
		x = (S/1000) * (y/5)
		"""
		if max_range_v <= 0:
			raise ValueError("max_range_v 必须为正")
		s = max(0, int(reg_value) & 0xFFFF)
		return (s / 1000.0) * (max_range_v / 5.0)

	# -------------------- 高层封装 --------------------
	def set_voltage(
		self,
		voltage_v: float,
		max_range_v: float,
		reg: int = 0x0001,
		retries: int = 2,
	) -> int:
		"""按协议设置电压，返回写入的寄存器值（整型）。"""
		value = self.voltage_to_register(voltage_v, max_range_v)
		last_err: Optional[Exception] = None
		for _ in range(max(1, retries)):
			try:
				self.write_register(reg, value)
				return value
			except Exception as e:  # noqa: BLE001
				last_err = e
				time.sleep(0.05)
		if last_err:
			raise last_err
		return value

	def read_voltage(
		self,
		max_range_v: float,
		reg: int = 0x0001,
		retries: int = 2,
	) -> float:
		"""读取寄存器并换算为电压值（单位：V）。"""
		last_err: Optional[Exception] = None
		for _ in range(max(1, retries)):
			try:
				vals = self.read_register(reg, 1)
				return self.register_to_voltage(vals[0], max_range_v)
			except Exception as e:  # noqa: BLE001
				last_err = e
				time.sleep(0.05)
		if last_err:
			raise last_err
		return 0.0

	# -------------------- 资源管理 --------------------
	def close(self) -> None:
		if self.ser and self.ser.is_open:
			self.ser.close()

	def __enter__(self) -> "DigitalPowerController":
		return self

	def __exit__(self, exc_type, exc, tb) -> None:  # noqa: D401
		self.close()


if __name__ == "__main__":
	# 使用示例（请根据实际端口与量程修改）：
	# 注意：运行前请确认已连接设备，且串口可用。
	# from time import sleep
	# with DigitalPowerController(port="COM3", addr=0x01) as dp:
	#     # 写 12V，量程 36V
	#     s = dp.set_voltage(12.0, max_range_v=36.0)
	#     print(f"写入寄存器值: {s}")
	#     # 读取电压
	#     v = dp.read_voltage(max_range_v=36.0)
	#     print(f"读取电压: {v:.3f} V")
	pass

