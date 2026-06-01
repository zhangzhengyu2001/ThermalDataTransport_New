#!/usr/bin/env python3
"""
面向对象的 Modbus-RTU 上位机工具（用于读取气压与设置目标气压）

- 设备串口：STM32 端 UART1（115200-8-N-1）
- 设备地址：0x01
- 支持：
    - 读保持寄存器（功能码 0x03）：
        * 0x0000~0x0001：滤波气压 get_filter_pressure()，float32（32 位大端，寄存器高字在前）
        * 0x0002~0x0003：air_pressure，uint32（32 位大端，寄存器高字在前）
    - 写单个寄存器（功能码 0x06）：
        * 0x0010（高 16 位）、0x0011（低 16 位）共同设置目标气压（uint32，单位：Pa）

示例（PowerShell）：
    # 安装依赖
    # python -m pip install pyserial

    # 读取当前气压
    # python tools/modbus_host.py --port COM3 read

    # 设置目标气压为 50000 Pa
    # python tools/modbus_host.py --port COM3 set-target --value 50000
"""

import argparse
import struct
import sys
import time
from typing import Tuple, List, Optional

import serial


# 寄存器映射
REG_FILTER_PRESSURE_HI = 0x0000  # 0x0000~0x0001 float32（滤波气压）
REG_AIR_PRESSURE_HI    = 0x0002  # 0x0002~0x0003 uint32（原始气压）
REG_TARGET_PRESSURE_HI = 0x0010  # 0x0010~0x0011 uint32（目标气压，写/读）
REG_CTRL_UPDATE        = 0x0020  # 控制/状态寄存器：写非零触发刷新；读返回 0/1 标志（现阶段可不用）
REG_START_PID          = 0x0021  # PID 启停标志：0=仅检测,1=控制
REG_FLOW_CMD_HI        = 0x0030  # 0x0030~0x0031 float32（MFC 开度指令 g_valve_flow_cmd）
REG_PID_KP_HI          = 0x0100  # 0x0100~0x0101 float32 kp
REG_PID_KI_HI          = 0x0102  # 0x0102~0x0103 float32 ki
REG_PID_KD_HI          = 0x0104  # 0x0104~0x0105 float32 kd
REG_PID_MAX_I_HI       = 0x0106  # 0x0106~0x0107 float32 maxIntegral
REG_PID_OUT_MIN_HI     = 0x0108  # 0x0108~0x0109 float32 outMin
REG_PID_OUT_MAX_HI     = 0x010A  # 0x010A~0x010B float32 outMax


class ModbusPressureClient:
    """
    Modbus-RTU 客户端（面向对象），用于与 STM32 设备交互。
    - 默认 115200-8-N-1，设备地址 0x01。
    - 提供读取滤波气压/原始气压、设置目标气压的接口。
    """

    def __init__(self, port: str, *, baudrate: int = 115200, timeout: float = 0.5, device_addr: int = 0x01):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.addr = device_addr
        self._ser: Optional[serial.Serial] = None

    # -------------------- 公共接口 --------------------
    def open(self) -> None:
        """打开串口。"""
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

    def close(self) -> None:
        """关闭串口。"""
        if self._ser:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def read_pressures(self) -> Tuple[float, int]:
        """读取滤波气压（float32, Pa）与原始气压（uint32, Pa）。"""
        regs = self.read_registers(REG_FILTER_PRESSURE_HI, 4)
        filtered = self._parse_be_f32_from_regs(regs[0], regs[1])
        air_u32 = self._parse_be_u32_from_regs(regs[2], regs[3])
        return filtered, air_u32

    # --- 新接口：写目标气压（float 或 int），供后端调用 ---
    def write_target_pressure(self, value_pa: float) -> None:
        """写入目标气压（float 或 int，单位 Pa）。"""
        self.set_target_pressure(int(value_pa))

    def set_target_pressure(self, value_pa: int) -> None:
        """设置目标气压（单位：Pa，0..4294967295）。"""
        if not (0 <= value_pa <= 0xFFFFFFFF):
            raise ValueError("目标气压需在 0..4294967295 范围内（Pa）")
        hi = (value_pa >> 16) & 0xFFFF
        lo = value_pa & 0xFFFF
        # 一次性写入两个寄存器（功能码 0x10）
        self.write_multiple_registers(REG_TARGET_PRESSURE_HI, [hi, lo])

    def get_target_pressure(self) -> int:
        """读取当前目标气压（uint32, Pa）。若固件支持读取 0x0010~0x0011。"""
        regs = self.read_registers(REG_TARGET_PRESSURE_HI, 2)
        return self._parse_be_u32_from_regs(regs[0], regs[1])

    def trigger_pressure_update(self) -> None:
        """触发设备执行滤波气压刷新：写控制寄存器 0x0020 为 1。"""
        self.write_single_register(REG_CTRL_UPDATE, 1)

    def clear_pressure_update_flag(self) -> None:
        """清除更新标志：写控制寄存器 0x0020 为 0。"""
        self.write_single_register(REG_CTRL_UPDATE, 0)

    def read_pressure_update_flag(self) -> int:
        """读取更新标志 (0/1)。"""
        regs = self.read_registers(REG_CTRL_UPDATE, 1)
        return regs[0] & 0xFFFF

    def trigger_and_read(self) -> Tuple[float, int]:
        """单步触发刷新并读取最新滤波/原始气压。"""
        self.trigger_pressure_update()
        # 可选延时，视刷新耗时决定；此处简单等待 10ms
        time.sleep(0.01)
        return self.read_pressures()

    # --- 新接口：写 PID 参数到 STM32 ---
    def _float_to_be_regs(self, value: float) -> Tuple[int, int]:
        b = struct.pack('>f', float(value))
        u32 = struct.unpack('>I', b)[0]
        hi = (u32 >> 16) & 0xFFFF
        lo = u32 & 0xFFFF
        return hi, lo

    def write_pid_params(self, *, kp: float, ki: float, kd: float,
                          max_i: float, out_min: float, out_max: float) -> None:
        """将 PID 参数写入 STM32 的 0x0100~0x010B 寄存器区。"""
        regs: List[int] = []
        for v in (kp, ki, kd, max_i, out_min, out_max):
            hi, lo = self._float_to_be_regs(v)
            regs.extend([hi, lo])
        # 从 0x0100 连续写 12 个寄存器
        self.write_multiple_registers(REG_PID_KP_HI, regs)

    def write_start_pid(self, flag: int) -> None:
        """写入 PID 启停标志：0=仅检测模式, 1=控制模式。"""
        self.write_single_register(REG_START_PID, 1 if flag else 0)

    def read_all_states(self) -> Tuple[float, int, float, float, int]:
        """一次性读取滤波气压、原始气压、目标气压、MFC 开度和 start_pid。

        返回: (filtered(float), air(uint32), target(float), flow_cmd(float), start_pid(int))
        """
        # 1) 读取滤波气压与原始气压: 0x0000..0x0003
        regs = self.read_registers(REG_FILTER_PRESSURE_HI, 4)
        filtered = self._parse_be_f32_from_regs(regs[0], regs[1])
        air_u32 = self._parse_be_u32_from_regs(regs[2], regs[3])

        # 2) 单独读取目标气压: 0x0010..0x0011
        t_regs = self.read_registers(REG_TARGET_PRESSURE_HI, 2)
        target_u32 = self._parse_be_u32_from_regs(t_regs[0], t_regs[1])
        target = float(target_u32)

        # 3) 读取 start_pid: 从 0x0020 开始连续取两个寄存器，第二个为 0x0021
        s_regs = self.read_registers(REG_CTRL_UPDATE, 2)
        start_pid = s_regs[1] & 0xFFFF

        # 4) 读取 MFC 开度指令（0~1），映射在 0x0030~0x0031
        try:
            flow_regs = self.read_registers(REG_FLOW_CMD_HI, 2)
            flow_cmd = self._parse_be_f32_from_regs(flow_regs[0], flow_regs[1])
        except Exception:
            flow_cmd = 0.0

        return filtered, air_u32, target, flow_cmd, start_pid

    # -------------------- 基础 Modbus 操作 --------------------
    def read_registers(self, start: int, count: int) -> List[int]:
        """读保持寄存器。返回 16 位寄存器列表。"""
        if not (0 <= start <= 0xFFFF and 1 <= count <= 0x7D):
            raise ValueError("起始地址/数量非法")
        pdu = bytes([
            self.addr,
            0x03,
            (start >> 8) & 0xFF, start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF,
        ])
        crc = self._crc16_modbus(pdu)
        req = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # CRC 低字节在前

        expect = 3 + (count * 2) + 2  # addr,func,byteCount + data + crc2
        rsp = self._send_and_recv_exact(req, expect)

        # 基本校验
        if rsp[0] != self.addr or rsp[1] != 0x03:
            raise IOError(f"响应头异常: {rsp[:2].hex()}")
        if rsp[2] != count * 2:
            raise IOError(f"字节数不匹配: {rsp[2]} != {count*2}")
        calc = self._crc16_modbus(rsp[:-2])
        rx_crc = rsp[-2] | (rsp[-1] << 8)
        if calc != rx_crc:
            raise IOError("CRC 校验失败（读）")

        regs: List[int] = []
        data = rsp[3:-2]
        for i in range(0, len(data), 2):
            hi = data[i]
            lo = data[i + 1]
            regs.append((hi << 8) | lo)
        return regs

    def write_single_register(self, reg: int, value: int) -> None:
        """写单个寄存器（功能码 0x06）。"""
        if not (0 <= reg <= 0xFFFF and 0 <= value <= 0xFFFF):
            raise ValueError("寄存器/数值非法")
        pdu = bytes([
            self.addr,
            0x06,
            (reg >> 8) & 0xFF, reg & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF,
        ])
        crc = self._crc16_modbus(pdu)
        req = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

        # 回显应与请求完全一致
        rsp = self._send_and_recv_exact(req, 8)
        if rsp != req:
            raise IOError("写单寄存器回显不一致")

    def write_multiple_registers(self, start: int, values: List[int]) -> None:
        """写多个寄存器（功能码 0x10）。values 为 16 位列表。"""
        if not (0 <= start <= 0xFFFF):
            raise ValueError("起始地址非法")
        if not (1 <= len(values) <= 0x7B):
            raise ValueError("写入寄存器数量超限")
        for v in values:
            if not (0 <= v <= 0xFFFF):
                raise ValueError("寄存器数值必须为 0..65535")

        qty = len(values)
        bc = qty * 2
        pdu = bytearray([
            self.addr,
            0x10,
            (start >> 8) & 0xFF, start & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF,
            bc & 0xFF,
        ])
        for v in values:
            pdu.append((v >> 8) & 0xFF)
            pdu.append(v & 0xFF)
        crc = self._crc16_modbus(bytes(pdu))
        req = bytes(pdu) + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

        # 响应固定 8 字节：addr,0x10,start(2),qty(2),CRC(2)
        rsp = self._send_and_recv_exact(req, 8)
        # 基本校验
        if rsp[0] != self.addr or rsp[1] != 0x10:
            raise IOError("写多寄存器响应头异常")
        if not (rsp[2] == ((start >> 8) & 0xFF) and rsp[3] == (start & 0xFF)):
            raise IOError("写多寄存器起始地址不匹配")
        if not (rsp[4] == ((qty >> 8) & 0xFF) and rsp[5] == (qty & 0xFF)):
            raise IOError("写多寄存器数量不匹配")
        calc = self._crc16_modbus(rsp[:-2])
        rx_crc = rsp[-2] | (rsp[-1] << 8)
        if calc != rx_crc:
            raise IOError("写多寄存器 CRC 校验失败")

    # -------------------- 内部工具函数 --------------------
    def _ensure_open(self) -> serial.Serial:
        if not self._ser or not self._ser.is_open:
            self.open()
        assert self._ser is not None
        return self._ser

    def _send_and_recv_exact(self, req: bytes, expect_len: int) -> bytes:
        ser = self._ensure_open()
        ser.reset_input_buffer()
        ser.write(req)
        ser.flush()

        deadline = time.time() + (ser.timeout or 1.0)
        buf = bytearray()
        while len(buf) < expect_len and time.time() < deadline:
            chunk = ser.read(expect_len - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.005)
        if len(buf) != expect_len:
            raise TimeoutError(f"串口超时：期望 {expect_len} 字节，实际 {len(buf)} 字节")
        return bytes(buf)

    @staticmethod
    def _crc16_modbus(data: bytes) -> int:
        """计算 Modbus RTU CRC16（多项式 0xA001）。"""
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    @staticmethod
    def _parse_be_u32_from_regs(reg_hi: int, reg_lo: int) -> int:
        return ((reg_hi & 0xFFFF) << 16) | (reg_lo & 0xFFFF)

    @staticmethod
    def _parse_be_f32_from_regs(reg_hi: int, reg_lo: int) -> float:
        u32 = ModbusPressureClient._parse_be_u32_from_regs(reg_hi, reg_lo)
        b = struct.pack('>I', u32)
        return struct.unpack('>f', b)[0]


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="STM32 气压监测 Modbus 上位机（面向对象实现）")
    parser.add_argument('--port', required=True, help='串口号，例如 COM3')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率，默认 115200')
    parser.add_argument('--timeout', type=float, default=0.5, help='串口超时（秒），默认 0.5')
    parser.add_argument('--addr', type=lambda x: int(x, 0), default=0x01, help='设备地址，默认 0x01')

    sub = parser.add_subparsers(dest='cmd', required=True)

    sub.add_parser('read', help='读取滤波气压(float32) 和 原始气压(uint32)')

    p_set = sub.add_parser('set-target', help='设置目标气压（uint32 Pa）')
    p_set.add_argument('--value', type=int, required=True, help='目标气压 Pa (0..4294967295)')

    p_get = sub.add_parser('get-target', help='读取目标气压（uint32 Pa）')

    sub.add_parser('trigger-update', help='触发设备刷新滤波气压')
    sub.add_parser('read-flag', help='读取刷新的完成标志 (0/1)')
    sub.add_parser('clear-flag', help='清除刷新完成标志（写0）')
    sub.add_parser('trigger-read', help='触发刷新并立即读取气压')

    args = parser.parse_args(argv)

    try:
        with ModbusPressureClient(args.port, baudrate=args.baudrate, timeout=args.timeout, device_addr=args.addr) as cli:
            if args.cmd == 'read':
                filtered, air = cli.read_pressures()
                print(f"滤波气压: {filtered:.3f} Pa, 原始气压: {air} Pa")
            elif args.cmd == 'set-target':
                cli.set_target_pressure(args.value)
                print(f"设置目标气压成功: {args.value} Pa")
            elif args.cmd == 'get-target':
                v = cli.get_target_pressure()
                print(f"当前目标气压: {v} Pa")
            elif args.cmd == 'trigger-update':
                cli.trigger_pressure_update()
                print("已发送刷新触发指令 (0x0020=1)")
            elif args.cmd == 'read-flag':
                flag = cli.read_pressure_update_flag()
                print(f"刷新标志: {flag}")
            elif args.cmd == 'clear-flag':
                cli.clear_pressure_update_flag()
                print("已清除刷新标志 (0x0020=0)")
            elif args.cmd == 'trigger-read':
                filtered, air = cli.trigger_and_read()
                print(f"触发后滤波气压: {filtered:.3f} Pa, 原始气压: {air} Pa")
            else:
                parser.error('未知命令')
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
