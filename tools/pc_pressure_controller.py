#!/usr/bin/env python3
"""
pc_pressure_controller.py
上位机闭环控制示例：
- 通过 Modbus 读取 STM32 的滤波气压 filtered_pressure
- 使用简单 PID 算法在 PC 端计算阀门开度 (0~1)
- 通过 MFC.py 驱动 MFC (地址32, 波特率9600) 实现气压闭环

依赖：
    pip install pyserial

用法示例（PowerShell）：
    python tools\pc_pressure_controller.py \
        --stm32-port COM3 --stm32-baud 115200 --stm32-addr 0x01 \
        --mfc-port COM4 --mfc-baud 9600 --mfc-addr 32 \
        --target 50000 --period 0.1 --duration 0
"""

import time
import argparse
import serial

from modbus_host import ModbusPressureClient
import MFC


class SimplePID:
    def __init__(self, kp, ki, kd, max_i, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_i = max_i
        self.out_min = out_min
        self.out_max = out_max
        self.last_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0

    def step(self, ref, fb):
        """单步 PID 计算，返回输出"""
        error = ref - fb
        p = self.kp * error
        self.integral += self.ki * error
        # 积分限幅
        if self.integral > self.max_i:
            self.integral = self.max_i
        elif self.integral < -self.max_i:
            self.integral = -self.max_i
        d = self.kd * (error - self.last_error)
        self.last_error = error
        out = p + self.integral + d
        # 输出限幅
        if out > self.out_max:
            out = self.out_max
        elif out < self.out_min:
            out = self.out_min
        return out


def main():
    parser = argparse.ArgumentParser(description="PC 端气压 + MFC 闭环控制示例")
    parser.add_argument("--stm32-port", required=True, help="STM32 Modbus 串口，例如 COM3")
    parser.add_argument("--stm32-baud", type=int, default=115200, help="STM32 UART1 波特率，默认 115200")
    parser.add_argument("--stm32-addr", type=lambda x: int(x, 0), default=0x01, help="Modbus 设备地址，默认 0x01")

    parser.add_argument("--mfc-port", required=True, help="MFC 串口，例如 COM4")
    parser.add_argument("--mfc-baud", type=int, default=9600, help="MFC 波特率，默认 9600")
    parser.add_argument("--mfc-addr", type=int, default=32, help="MFC 地址，默认 32")

    parser.add_argument("--target", type=float, required=True, help="目标气压 (Pa)")
    parser.add_argument("--period", type=float, default=0.1, help="控制周期 (秒)，默认 0.1")
    parser.add_argument("--duration", type=float, default=0, help="运行时长 (秒)，0 表示一直运行")

    args = parser.parse_args()

    # 简单 PID 参数（需要根据系统实际调试）
    pid = SimplePID(
        kp=0.001,    # 比例系数
        ki=0.0001,   # 积分系数
        kd=0.0,      # 微分系数
        max_i=10000, # 积分上限
        out_min=0.0,
        out_max=1.0  # 输出映射到 0~1 流量指令
    )

    # 打开 STM32 Modbus 串口
    modbus_cli = ModbusPressureClient(
        port=args.stm32_port,
        baudrate=args.stm32_baud,
        timeout=0.5,
        device_addr=args.stm32_addr
    )

    # 打开 MFC 串口
    mfc_ser = serial.Serial(
        port=args.mfc_port,
        baudrate=args.mfc_baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
    )

    try:
        modbus_cli.open()
        print(f"已连接 STM32: {args.stm32_port}, Modbus addr=0x{args.stm32_addr:X}")
        print(f"已连接 MFC: {args.mfc_port}, addr={args.mfc_addr}, baud={args.mfc_baud}")
        print(f"目标气压: {args.target} Pa")

        start_time = time.time()
        while True:
            loop_begin = time.time()

            # 若设置了 duration 且超时则退出
            if args.duration > 0 and (loop_begin - start_time) > args.duration:
                print("到达设定运行时长，退出控制循环")
                break

            try:
                # 读取滤波气压 filtered 和原始气压 air
                filtered, air = modbus_cli.read_pressures()
            except Exception as e:
                print(f"[WARN] 读取气压失败: {e}")
                time.sleep(args.period)
                continue

            # 以滤波气压作为反馈
            feedback = filtered
            ref = args.target

            # PID 计算 -> 0~1 流量值
            flow_cmd = pid.step(ref, feedback)

            # 发送到 MFC
            try:
                MFC.setValveFlowRate(mfc_ser, args.mfc_addr, flow_cmd)
            except Exception as e:
                print(f"[WARN] 发送 MFC 指令失败: {e}")

            # （可选）读取即时流量做监控
            try:
                _, flow_meas = MFC.getInstantFlowRate(mfc_ser, args.mfc_addr)
            except Exception:
                flow_meas = None

            if flow_meas is not None:
                print(f"ref={ref:.1f} Pa, fb={feedback:.1f} Pa, cmd={flow_cmd:.3f}, mfc={flow_meas:.3f}")
            else:
                print(f"ref={ref:.1f} Pa, fb={feedback:.1f} Pa, cmd={flow_cmd:.3f}")

            # 控制周期对齐
            elapsed = time.time() - loop_begin
            sleep_t = args.period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    finally:
        try:
            # 退出前可安全关阀
            MFC.setValveFlowRate(mfc_ser, args.mfc_addr, 0.0)
        except Exception:
            pass
        mfc_ser.close()
        modbus_cli.close()
        print("串口已关闭，控制结束")


if __name__ == "__main__":
    main()
