#!/usr/bin/env python3
"""backend_server.py

后端服务（新版架构）：
- 通过 Modbus 直接读取 STM32 内部 PID 控制后的滤波气压等寄存器
- 所有 PID 计算和对 MFC 的控制均在 STM32 上完成
- 上位机仅负责：目标气压、PID 参数、模式（仅检测 / 控制）的下发，以及数据可视化

运行示例：
    uvicorn tools.backend_server:app --reload --host 0.0.0.0 --port 8000
"""

import asyncio
import threading
import time
from typing import Optional, List, Dict, Any
import os
import csv
import json
import urllib.request
import urllib.error

import serial
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from tools.modbus_host import ModbusPressureClient
from modbus.digital_power import DigitalPowerController

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None


# ---- 配置加载 ----
_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")


def _deep_update(dst: Dict[str, Any], src: Dict[str, Any]) -> None:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_update(dst[k], v)
        else:
            dst[k] = v


DEFAULT_CONFIG: Dict[str, Any] = {
    "pressure": {
        "stm32": {
            "addr": 0x01,
            "baudrate": 115200,
        },
        "pid": {
            "kp": 0.00005,
            "ki": 0.00000078,
            "kd": 0.000003,
            "max_i": 8000.0,
            "out_min": 0.0,
            "out_max": 1.0,
        },
        "period": 0.1,
        "history_window": 60.0,
    },
    "power": {
        "addr": 0x01,
        "baudrate": 19200,
        # 数字电源的电压量程（单位：V），用于电压 <-> 寄存器换算
        # 目前使用的电源量程10kV，请根据实际设备调整
        "max_voltage": 10000,
    },
}


def load_config() -> Dict[str, Any]:
    cfg: Dict[str, Any] = json.loads(json.dumps(DEFAULT_CONFIG))
    if os.path.exists(_CONFIG_PATH):
        try:
            with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
                user_cfg = json.load(f)
            if isinstance(user_cfg, dict):
                _deep_update(cfg, user_cfg)
        except Exception as e:  # noqa: BLE001
            print(f"[CFG] 加载配置失败，使用默认值: {e}")
    else:
        print(f"[CFG] 未找到配置文件 {_CONFIG_PATH}，使用默认配置")
    return cfg


CONFIG = load_config()


def reload_pid_from_config() -> None:
    """从配置文件重新加载气压 PID 参数到当前上下文。

    不主动写入 STM32，仅更新内存中的 PID 配置，
    由调用方决定是否调用 apply_pid_to_stm32() 下发到设备。
    """
    global CONFIG
    CONFIG = load_config()
    pressure_cfg = CONFIG.get("pressure", {})
    pid_cfg = pressure_cfg.get("pid", {})
    # 若配置缺失某些字段，则保留当前 ctx 中的值
    ctx.pid_kp = float(pid_cfg.get("kp", ctx.pid_kp))
    ctx.pid_ki = float(pid_cfg.get("ki", ctx.pid_ki))
    ctx.pid_kd = float(pid_cfg.get("kd", ctx.pid_kd))
    ctx.pid_max_i = float(pid_cfg.get("max_i", ctx.pid_max_i))
    ctx.pid_out_min = float(pid_cfg.get("out_min", ctx.pid_out_min))
    ctx.pid_out_max = float(pid_cfg.get("out_max", ctx.pid_out_max))

# ---- MSHTTPFastAPI 远程调用辅助函数 ----
# 提供一个通用的 HTTP POST 辅助函数，用于调用远程原生MSFileReader的接口，简化后续具体接口调用的实现。
def _ms_http_post(base_url: str, func: str, payload: Dict[str, Any], timeout: float = 5.0) -> Dict[str, Any]:
    """调用远程 MSHTTPFastAPI 的通用 POST 辅助函数。"""
    url = f"{base_url}/{func}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:  # noqa: S310
        text = resp.read().decode("utf-8")
    return json.loads(text)

# 针对 MSHTTPFastAPI 的 JSON 接口调用，提供一个更具体的辅助函数，便于后续调用时更清晰地表达意图。
# 目前主要用于远程文件选择器
def _ms_http_post_json(base_url: str, path: str, payload: Dict[str, Any], timeout: float = 5.0) -> Dict[str, Any]:
    """调用远程 MSHTTPFastAPI 的自定义 JSON 接口。"""
    if not path.startswith("/"):
        path = "/" + path
    url = f"{base_url}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:  # noqa: S310
        text = resp.read().decode("utf-8")
    return json.loads(text)


def _ms_refresh_and_get_end_time(server: str, raw_path: str) -> str:
    """在 MSHTTPFastAPI 上打开/刷新指定 raw 文件，并返回最新质谱时间。

    保留该函数，用于在未显式打开文件时的一次性 Open+Refresh+GetEndTime 调用。
    """
    host = server.strip()
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]

    base_url = f"http://{host}:8899"

    # 1) 打开文件（容错：若已打开会重新打开）
    open_payload = {"argsNames": [], "args": [raw_path]}
    open_res = _ms_http_post(base_url, "Open", open_payload)
    if open_res.get("DataType") != 2:
        raise RuntimeError(f"MS Open 失败: {open_res.get('error')}")

    # 2) 调用 RefreshViewOfFile（通常无参数）
    refresh_payload = {"argsNames": [], "args": []}
    refresh_res = _ms_http_post(base_url, "RefreshViewOfFile", refresh_payload)
    if refresh_res.get("DataType") != 2:
        raise RuntimeError(f"MS RefreshViewOfFile 失败: {refresh_res.get('error')}")

    # 3) 获取最新结束时间
    end_payload = {"argsNames": [], "args": []}
    end_res = _ms_http_post(base_url, "GetEndTime", end_payload)
    if end_res.get("DataType") != 2:
        raise RuntimeError(f"MS GetEndTime 失败: {end_res.get('error')}")
    res_list = end_res.get("res") or []
    if not res_list:
        raise RuntimeError("MS GetEndTime 返回空结果")
    ms_end_time = str(res_list[0])
    return ms_end_time


def _ms_refresh_and_get_end_time_no_open(server: str, raw_path: str) -> str:
    """仅调用 RefreshViewOfFile + GetEndTime，不再次 Open 文件。"""
    host = server.strip()
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]

    base_url = f"http://{host}:8899"

    refresh_payload = {"argsNames": [], "args": []}
    refresh_res = _ms_http_post(base_url, "RefreshViewOfFile", refresh_payload)
    if refresh_res.get("DataType") != 2:
        raise RuntimeError(f"MS RefreshViewOfFile 失败: {refresh_res.get('error')}")

    end_payload = {"argsNames": [], "args": []}
    end_res = _ms_http_post(base_url, "GetEndTime", end_payload)
    if end_res.get("DataType") != 2:
        raise RuntimeError(f"MS GetEndTime 失败: {end_res.get('error')}")
    res_list = end_res.get("res") or []
    if not res_list:
        raise RuntimeError("MS GetEndTime 返回空结果")
    return str(res_list[0])


def _ms_get_last_spectrum_number(server: str) -> int:
    """调用远程 GetLastSpectrumNumber，返回最新谱图号。"""
    host = server.strip()
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]

    base_url = f"http://{host}:8899"
    payload: Dict[str, Any] = {"argsNames": [], "args": []}
    res = _ms_http_post(base_url, "GetLastSpectrumNumber", payload)
    if res.get("DataType") != 2:
        raise RuntimeError(f"MS GetLastSpectrumNumber 失败: {res.get('error')}")
    lst = res.get("res") or []
    if not lst:
        raise RuntimeError("MS GetLastSpectrumNumber 返回空结果")
    return int(lst[0])


def _append_ms_log(
    server: str,
    raw_path: str,
    ms_end_time: str,
    target_pressure: float,
    voltage: float,
    actual_pressure: float,
    mfc_opening: float,
    last_spectrum_number: str,
) -> str:
    """将当前气压、电压、实际气压、MFC 开度与质谱信息写入日志文件，返回日志文件路径。

    last_spectrum_number 使用字符串存储，便于在获取失败时写入空字符串。
    """
    os.makedirs("data", exist_ok=True)
    log_path = os.path.join("data", "ms_log.csv")
    is_new = not os.path.exists(log_path)
    now_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    raw_name = os.path.basename(raw_path)
    row = [
        now_str,
        server,
        raw_path,
        raw_name,
        target_pressure,
        voltage,
        actual_pressure,
        mfc_opening,
        last_spectrum_number,
        ms_end_time,
    ]

    with open(log_path, "a", newline="", encoding="utf-8") as fh:
        import csv as _csv

        writer = _csv.writer(fh)
        if is_new:
            writer.writerow([
                "timestamp",
                "server",
                "raw_path",
                "raw_name",
                "target_pressure",
                "voltage",
                "actual_pressure",
                "mfc_opening",
                "last_spectrum_number",
                "ms_end_time",
            ])
        writer.writerow(row)

    return log_path


def _ms_log_current_if_open() -> str:
    """记录当前气压、电压状态；若远程 raw 已打开，则先刷新质谱并写入最新时间。"""
    server = ctx.ms_server or ""
    raw_path = ctx.ms_raw_path or ""
    ms_end_time = ""
    last_spec = ""
    if ctx.ms_server and ctx.ms_raw_path and ctx.ms_open:
        try:
            ms_end_time = _ms_refresh_and_get_end_time_no_open(ctx.ms_server, ctx.ms_raw_path)
        except Exception as e:  # noqa: BLE001
            print(f"[MSLOG] 刷新质谱失败: {e}")
        try:
            last_spec_num = _ms_get_last_spectrum_number(ctx.ms_server)
            last_spec = str(last_spec_num)
        except Exception as e:  # noqa: BLE001
            print(f"[MSLOG] 获取最新谱图号失败: {e}")
    return _append_ms_log(
        server=server,
        raw_path=raw_path,
        ms_end_time=ms_end_time,
        target_pressure=ctx.current_target,
        voltage=ctx.current_voltage,
        actual_pressure=ctx.current_filtered,
        mfc_opening=ctx.current_flow_cmd,
        last_spectrum_number=last_spec,
    )
class ControlContext:
    def __init__(self):
        # STM32 Modbus 串口配置
        pressure_cfg = CONFIG.get("pressure", {})
        stm32_cfg = pressure_cfg.get("stm32", {})
        self.stm32_port: Optional[str] = None
        self.stm32_baud: int = int(stm32_cfg.get("baudrate", 115200))
        self.stm32_addr: int = int(stm32_cfg.get("addr", 0x01))

        # 采样周期（秒），仅用于上位机轮询刷新频率；
        # 实际控制周期在 STM32 固定为 100 ms。
        self.period: float = float(pressure_cfg.get("period", 0.1))

        # 当前状态（从 STM32 寄存器读出）
        self.current_filtered: float = 0.0
        self.current_air: int = 0
        self.current_target: float = 0.0
        self.current_flow_cmd: float = 0.0
        self.current_start_pid: int = 0  # 0=仅检测, 1=控制模式

        # 本地缓存的 PID 参数，仅用于在向 STM32 写入；实际控制参数以 STM32 为准。
        pid_cfg = pressure_cfg.get("pid", {})
        self.pid_kp: float = float(pid_cfg.get("kp", 0.00005))
        self.pid_ki: float = float(pid_cfg.get("ki", 0.00000078))
        self.pid_kd: float = float(pid_cfg.get("kd", 0.000003))
        self.pid_max_i: float = float(pid_cfg.get("max_i", 8000.0))
        self.pid_out_min: float = float(pid_cfg.get("out_min", 0.0))
        self.pid_out_max: float = float(pid_cfg.get("out_max", 1.0))

        # 历史数据（仅缓存在内存中，按窗口裁剪），点击“保存数据”时一次性写入文件
        self.history_window: float = float(pressure_cfg.get("history_window", 60.0))  # 保留时长 (秒)
        # (timestamp, filtered, air, target, flow_cmd)
        self._history: List[tuple] = []

        # 电流监控（独立串口，ASCII 文本协议，仅被动接收）
        # 默认固定波特率 9600，无需前端配置
        self.current_port: Optional[str] = None
        self.current_baud: int = 9600
        self.current_value_a: float = 0.0  # 最新电流值，单位 A
        # (timestamp, current_a)
        self._current_history: List[tuple] = []
        self._current_ser: Optional[serial.Serial] = None
        self._current_thread: Optional[threading.Thread] = None
        self._current_stop = threading.Event()

        # 串口与线程资源
        self._modbus_cli: Optional[ModbusPressureClient] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

        # 数字电源控制配置
        power_cfg = CONFIG.get("power", {})
        self.power_port: Optional[str] = None
        self.power_baud: int = int(power_cfg.get("baudrate", 19200))
        self.power_addr: int = int(power_cfg.get("addr", 0x01))
        self.power_max_voltage: float = float(power_cfg.get("max_voltage", 10000.0))
        self.current_voltage: float = 0.0
        self._power: Optional[DigitalPowerController] = None

        # 质谱远程文件状态
        self.ms_server: Optional[str] = None
        self.ms_raw_path: Optional[str] = None
        self.ms_open: bool = False

        # WebSocket 客户端列表
        self._ws_clients: List[WebSocket] = []
        self._ws_lock = asyncio.Lock()

        # WebSocket 统一广播线程（独立于各监控/控制线程）
        self._broadcast_stop = threading.Event()
        self._broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self._broadcast_thread.start()

    # ---- 串口操作 ----
    def open_stm32(self):
        if not self.stm32_port:
            raise RuntimeError("STM32 串口号未设置")
        if self._modbus_cli and self._modbus_cli._ser and self._modbus_cli._ser.is_open:
            return
        self._modbus_cli = ModbusPressureClient(
            port=self.stm32_port,
            baudrate=self.stm32_baud,
            timeout=0.5,
            device_addr=self.stm32_addr,
        )
        self._modbus_cli.open()

    def close_stm32(self):
        if self._modbus_cli:
            try:
                self._modbus_cli.close()
            except Exception:
                pass
            self._modbus_cli = None

    def apply_pid_to_stm32(self):
        """根据当前 PID 参数（通常来自配置文件）写入 STM32。"""
        if self._modbus_cli is None:
            raise RuntimeError("STM32 串口未打开")
        self._modbus_cli.write_pid_params(
            kp=self.pid_kp,
            ki=self.pid_ki,
            kd=self.pid_kd,
            max_i=self.pid_max_i,
            out_min=self.pid_out_min,
            out_max=self.pid_out_max,
        )

    # ---- 电流监控串口 ----
    def _parse_current_line(self, text: str) -> Optional[float]:
        """解析电流监控设备返回的一行 ASCII 文本，提取电流值（单位 A）。

        按设备固定格式直接按字节下标截取：

        - 第 2~9 个字节（0-based，包含 2 和 9）为数值字符串，例如 "+000.000"；
        - 第 10~11 个字节为单位字符串，例如 "uA"、" A" 等；

        示例::

            C:+000.000uA1:00.00000V\r\n

        解析失败返回 None。
        """

        try:
            # 确保长度足够
            if len(text) < 12:
                return None

            value_str = text[2:10].strip()
            unit_str = text[10:12].strip()
            if not value_str or not unit_str:
                return None

            try:
                value = float(value_str)
            except ValueError:
                return None

            if unit_str == "uA":
                mul = 1e-6
            elif unit_str == "mA":
                mul = 1e-3
            elif unit_str == " A" or unit_str == "A":
                # 例如 "A" 或 " A" 等，统一视为 A
                mul = 1.0
            else:
                return None

            return value * mul
        except Exception:
            return None

    def open_current(self):
        """打开电流监控串口，并启动接收线程。

        电流串口仅被动接收 ASCII 文本行，不发送任何查询指令。
        """

        if not self.current_port:
            raise RuntimeError("电流串口号未设置")
        if self._current_ser and self._current_ser.is_open:
            # 已经打开，直接返回
            return

        try:
            self._current_ser = serial.Serial(
                self.current_port,
                baudrate=self.current_baud,
                timeout=1.0,
            )
        except Exception as e:
            self._current_ser = None
            raise RuntimeError(f"打开电流串口失败: {e}") from e

        # 启动接收线程
        self._current_stop.clear()
        self._current_thread = threading.Thread(target=self._current_loop, daemon=True)
        self._current_thread.start()

    def close_current(self):
        """关闭电流串口与接收线程。"""

        self._current_stop.set()
        if self._current_thread:
            try:
                self._current_thread.join(timeout=2.0)
            except Exception:
                pass
            self._current_thread = None

        if self._current_ser:
            try:
                self._current_ser.close()
            except Exception:
                pass
            self._current_ser = None

    def _current_loop(self):
        """后台线程：循环从电流串口读取并解析数据。"""

        ser = self._current_ser
        if ser is None:
            return

        while not self._current_stop.is_set():
            try:
                line = ser.readline()
                if not line:
                    continue
                try:
                    text = line.decode("ascii", errors="ignore").strip()
                except Exception:
                    continue
                if not text:
                    continue

                value_a = self._parse_current_line(text)
                if value_a is None:
                    continue

                now_ts = time.time()
                self.current_value_a = value_a
                self._current_history.append((now_ts, value_a))
                cutoff = now_ts - self.history_window
                self._current_history = [item for item in self._current_history if item[0] >= cutoff]
            except Exception as e:
                # 读取或解析失败时，仅打印日志并稍作等待，避免线程退出
                print(f"[CURRENT] 读取或解析电流数据失败: {e}")
                time.sleep(0.5)

    # ---- 数字电源串口 ----
    def open_power(self):
        if not self.power_port:
            raise RuntimeError("电源串口号未设置")
        if self._power and self._power.ser and self._power.ser.is_open:
            return
        self._power = DigitalPowerController(
            port=self.power_port,
            addr=self.power_addr,
            baudrate=self.power_baud,
        )

    def close_power(self):
        if self._power:
            try:
                self._power.close()
            except Exception:
                pass
            self._power = None

    # ---- 控制线程 ----
    def start_control(self):
        if self._thread and self._thread.is_alive():
            return
        # 仅要求 STM32 串口已打开；
        if not self._modbus_cli:
            raise RuntimeError("请先打开 STM32 串口")
        # 每次开始新的控制过程时，清空历史缓冲区，避免混入上一次控制的数据
        self._history.clear()
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop_control(self):
        self._stop_flag.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        # 停止后不再更新 _history，此时缓冲区内容即为“停止前 history_window 秒内的数据”

    # ---- 数据快照保存 ----
    def save_snapshot(self):
        """将当前内存中的历史数据一次性写入 CSV 文件。

        注意：如果已经调用过 stop_control，则 _history 不会再增长，
        此时写出的就是“停止控制前 history_window 秒内”的数据。
        """
        if not self._history:
            print("[CTRL] 当前历史缓冲区为空，未生成文件")
            return

        os.makedirs("data", exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        filename = os.path.join("data", f"pressure_{ts}.csv")
        try:
            with open(filename, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow([
                    "timestamp",
                    "filtered_pressure",
                    "air_pressure",
                    "target_pressure",
                    "mfc_opening",
                ])
                for item in self._history:
                    writer.writerow(list(item))
            print(f"[CTRL] 已将 {len(self._history)} 条数据保存到 {filename}")
        except Exception as e:
            print(f"[CTRL] 保存快照失败: {e}")

    def _loop(self):
        while not self._stop_flag.is_set() and self._modbus_cli:
            t0 = time.time()
            try:
                # 直接从 STM32 读取当前滤波气压、原始气压、目标值与阀门开度
                filtered, air, target, flow_cmd, start_pid = self._modbus_cli.read_all_states()
                self.current_filtered = filtered
                self.current_air = air
                self.current_target = target
                self.current_flow_cmd = flow_cmd
                self.current_start_pid = start_pid
            except Exception as e:
                print(f"[CTRL] 读取气压失败: {e}")
                time.sleep(self.period)
                continue

            # 更新内存历史，仅按 history_window 做裁剪
            now_ts = time.time()
            # 记录当前滤波气压、原始气压、目标气压和 MFC 开度（流量指令）
            self._history.append(
                (now_ts, self.current_filtered, self.current_air, self.current_target, self.current_flow_cmd)
            )
            cutoff = now_ts - self.history_window
            # 丢弃窗口之外的旧数据
            self._history = [item for item in self._history if item[0] >= cutoff]

            elapsed = time.time() - t0
            sleep_t = self.period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ---- WebSocket 通知 ----
    def _broadcast_loop(self):
        """统一广播线程：定期将当前状态通过 WebSocket 推送给所有前端。

        所有监控/控制线程（气压、电流等）只负责更新 ctx 内部状态，
        不直接调用 _broadcast，从而避免多线程重复或竞争广播。
        """

        while not self._broadcast_stop.is_set():
            t0 = time.time()
            try:
                asyncio.run(self._broadcast())
            except Exception as e:  # noqa: BLE001
                print(f"[WS] 广播线程发送数据失败: {e}")

            elapsed = time.time() - t0
            # 使用 period 作为默认广播间隔；若非常小则兜底为 10ms
            sleep_t = self.period - elapsed
            if sleep_t < 0.01:
                sleep_t = 0.01
            time.sleep(sleep_t)

    async def register_ws(self, ws: WebSocket):
        async with self._ws_lock:
            self._ws_clients.append(ws)

    async def unregister_ws(self, ws: WebSocket):
        async with self._ws_lock:
            if ws in self._ws_clients:
                self._ws_clients.remove(ws)

    async def _broadcast(self):
        data = {
            "timestamp": time.time(),
            "filtered_pressure": self.current_filtered,
            "air_pressure": self.current_air,
            "target_pressure": self.current_target,
            "flow_cmd": self.current_flow_cmd,
            "current_a": self.current_value_a,
            "period": self.period,
            "mode": "control" if self.current_start_pid else "monitor",
            "pid": {
                "kp": self.pid_kp,
                "ki": self.pid_ki,
                "kd": self.pid_kd,
                "max_i": self.pid_max_i,
                "out_min": self.pid_out_min,
                "out_max": self.pid_out_max,
            },
        }
        async with self._ws_lock:
            dead = []
            for ws in self._ws_clients:
                try:
                    await ws.send_json(data)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                if ws in self._ws_clients:
                    self._ws_clients.remove(ws)


ctx = ControlContext()

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class TargetBody(BaseModel):
    target: float


class PIDBody(BaseModel):
    kp: float
    ki: float
    kd: float
    max_i: Optional[float] = None
    out_min: Optional[float] = None
    out_max: Optional[float] = None


class PeriodBody(BaseModel):
    period: float


class HistoryWindowBody(BaseModel):
    seconds: float


class ComConfigBody(BaseModel):
    port: str
    baudrate: int
    addr: Optional[int] = None


class CurrentComBody(BaseModel):
    """电流监控串口配置。

    仅需要端口号，波特率固定为 9600。"""

    port: str


class VoltageBody(BaseModel):
    voltage: float


class MsLogBody(BaseModel):
    server: Optional[str] = None  # MSHTTPFastAPI 服务器 IP 或主机名
    raw_path: Optional[str] = None  # .raw 文件完整路径


class MsFsRootsBody(BaseModel):
    server: Optional[str] = None  # MSHTTPFastAPI 服务器 IP 或主机名


class MsFsListBody(BaseModel):
    server: Optional[str] = None  # MSHTTPFastAPI 服务器 IP 或主机名
    path: str


class MsChroBody(BaseModel):
    mass_range: str  # 质荷比范围，例如 "100-200"
    start_time: Optional[float] = None  # EIC 起始时间 (min)
    end_time: Optional[float] = None  # EIC 结束时间 (min)


@app.post("/api/reload-pid")
async def api_reload_pid():
    """从配置文件重新加载 PID 参数并写入 STM32。

    用于在修改 config.json 后，手动将新的 PID 下发到 STM32。
    """
    # 先从配置文件刷新内存中的 PID 参数
    reload_pid_from_config()
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    # 将当前 PID 参数写入 STM32
    ctx.apply_pid_to_stm32()
    return {
        "pid": {
            "kp": ctx.pid_kp,
            "ki": ctx.pid_ki,
            "kd": ctx.pid_kd,
            "max_i": ctx.pid_max_i,
            "out_min": ctx.pid_out_min,
            "out_max": ctx.pid_out_max,
        }
    }


@app.get("/api/ports")
async def list_serial_ports():
    """返回当前可用串口列表，供前端选择。

    返回示例: [{"device": "COM3", "description": "USB-SERIAL"}, ...]
    """
    if list_ports is None:
        return {"ports": []}
    ports_info = []
    for p in list_ports.comports():
        ports_info.append({
            "device": p.device,
            "description": p.description,
        })
    return {"ports": ports_info}


@app.get("/api/state")
async def get_state():
    return {
        "target": ctx.current_target,
        "period": ctx.period,
        "pid": {
            "kp": ctx.pid_kp,
            "ki": ctx.pid_ki,
            "kd": ctx.pid_kd,
            "max_i": ctx.pid_max_i,
            "out_min": ctx.pid_out_min,
            "out_max": ctx.pid_out_max,
        },
        "stm32": {"port": ctx.stm32_port, "baud": ctx.stm32_baud, "addr": ctx.stm32_addr},
        "monitor_only": ctx.current_start_pid == 0,
        "history_window": ctx.history_window,
        "power": {
            "port": ctx.power_port,
            "baud": ctx.power_baud,
            "addr": ctx.power_addr,
            "max_voltage": ctx.power_max_voltage,
            "current_voltage": ctx.current_voltage,
        },
        "current": {
            "port": ctx.current_port,
            "baud": ctx.current_baud,
        },
        "ms": {
            "server": ctx.ms_server,
            "raw_path": ctx.ms_raw_path,
            "open": ctx.ms_open,
        },
    }


@app.post("/api/target")
async def set_target(body: TargetBody):
    t = float(body.target)
    # 写入 STM32 目标气压寄存器（0x0010/0x0011）
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_target_pressure(t)
    except Exception as e:
        print(f"[CTRL] 写入目标气压失败: {e}")
        return {"error": str(e)}
    ctx.current_target = t
    # 自动记录一条日志（若远程 raw 已打开则会刷新质谱）
    try:
        await asyncio.to_thread(_ms_log_current_if_open)
    except Exception as e:  # noqa: BLE001
        print(f"[MSLOG] 记录日志失败(设置目标气压): {e}")
    return {"target": ctx.current_target}


@app.post("/api/pid")
async def set_pid(body: PIDBody):
    # 数值合法性校验
    max_i = float(body.max_i) if body.max_i is not None else ctx.pid_max_i
    out_min = float(body.out_min) if body.out_min is not None else ctx.pid_out_min
    out_max = float(body.out_max) if body.out_max is not None else ctx.pid_out_max
    if not (max_i > 0):
        raise ValueError("max_i must > 0")
    if not (0.0 <= out_min < out_max <= 1.0):
        raise ValueError("require 0 <= out_min < out_max <= 1")

    ctx.pid_kp = float(body.kp)
    ctx.pid_ki = float(body.ki)
    ctx.pid_kd = float(body.kd)
    ctx.pid_max_i = max_i
    ctx.pid_out_min = out_min
    ctx.pid_out_max = out_max

    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")

    # 通过 Modbus 写入 STM32 端 PID 参数寄存器：
    ctx.apply_pid_to_stm32()
    return {
        "pid": {
            "kp": ctx.pid_kp,
            "ki": ctx.pid_ki,
            "kd": ctx.pid_kd,
            "max_i": ctx.pid_max_i,
            "out_min": ctx.pid_out_min,
            "out_max": ctx.pid_out_max,
        }
    }


@app.post("/api/period")
async def set_period(body: PeriodBody):
    if body.period <= 0:
        raise ValueError("period must > 0")
    ctx.period = float(body.period)
    return {"period": ctx.period}


@app.post("/api/history-window")
async def set_history_window(body: HistoryWindowBody):
    if body.seconds <= 0:
        raise ValueError("history window must > 0")
    ctx.history_window = float(body.seconds)
    # 清空已记录但超出窗口的数据
    now_ts = time.time()
    cutoff = now_ts - ctx.history_window
    ctx._history = [item for item in ctx._history if item[0] >= cutoff]
    ctx._current_history = [item for item in ctx._current_history if item[0] >= cutoff]
    return {"history_window": ctx.history_window}


@app.post("/api/open-stm32")
async def api_open_stm32(body: ComConfigBody):
    ctx.stm32_port = body.port
    ctx.stm32_baud = body.baudrate
    if body.addr is not None:
        ctx.stm32_addr = body.addr
    ctx.open_stm32()
    return {"ok": True, "port": ctx.stm32_port, "baud": ctx.stm32_baud, "addr": ctx.stm32_addr}


@app.post("/api/open-power")
async def api_open_power(body: ComConfigBody):
    """打开数字电源串口。

    前端仅需要提供串口号，波特率和地址从配置文件读取。
    """
    ctx.power_port = body.port
    # 波特率、地址均以配置文件为准，此处忽略 body 中的设置
    ctx.open_power()
    return {"ok": True, "port": ctx.power_port, "baud": ctx.power_baud, "addr": ctx.power_addr}


@app.post("/api/close-power")
async def api_close_power():
    ctx.close_power()
    return {"ok": True}


@app.post("/api/open-current")
async def api_open_current(body: CurrentComBody):
    """打开电流监控串口。

    协议为明文 ASCII，仅监听接收数据，不发送任何指令。"""

    ctx.current_port = body.port
    ctx.open_current()
    return {"ok": True, "port": ctx.current_port, "baud": ctx.current_baud}


@app.post("/api/close-current")
async def api_close_current():
    """关闭电流监控串口。"""

    ctx.close_current()
    return {"ok": True}


@app.post("/api/power/voltage")
async def api_set_power_voltage(body: VoltageBody):
    """设置数字电源输出电压。"""
    if ctx._power is None:
        raise RuntimeError("电源串口未打开")
    v = float(body.voltage)
    if v < 0:
        raise ValueError("目标电压必须 >= 0")
    if v > ctx.power_max_voltage:
        raise ValueError(f"目标电压不能超过量程 {ctx.power_max_voltage} V")
    reg = ctx._power.set_voltage(v, max_range_v=ctx.power_max_voltage)
    ctx.current_voltage = v
    # 自动记录一条日志（若远程 raw 已打开则会刷新质谱）
    try:
        await asyncio.to_thread(_ms_log_current_if_open)
    except Exception as e:  # noqa: BLE001
        print(f"[MSLOG] 记录日志失败(设置目标电压): {e}")
    return {"voltage": ctx.current_voltage, "register": reg}


@app.post("/api/ms-log")
async def api_ms_log(body: MsLogBody):
    """远程调用 MSHTTPFastAPI 刷新指定 raw 文件并写入日志。

    步骤：
    1. 在 MS 服务器上 Open(raw_path)
    2. 调用 RefreshViewOfFile
    3. 调用 GetEndTime 获取质谱最新时间
    4. 将当前目标气压、数字电源电压、raw 文件名和质谱时间写入本地日志
    """

    server = (body.server or ctx.ms_server or "").strip()
    raw_path = (body.raw_path or ctx.ms_raw_path or "").strip()
    if not server:
        raise RuntimeError("MS 服务器地址不能为空")
    if not raw_path:
        raise RuntimeError("raw 文件路径不能为空")

    # 若已经通过 /api/ms-open 打开相同文件，则只做 Refresh+GetEndTime
    if ctx.ms_open and ctx.ms_server == server and ctx.ms_raw_path == raw_path:
        ms_end_time = await asyncio.to_thread(_ms_refresh_and_get_end_time_no_open, server, raw_path)
    else:
        # 否则执行一次 Open+Refresh+GetEndTime，并视为当前打开文件
        ms_end_time = await asyncio.to_thread(_ms_refresh_and_get_end_time, server, raw_path)
        ctx.ms_server = server
        ctx.ms_raw_path = raw_path
        ctx.ms_open = True

    # 获取最新谱图号
    last_spec = ""
    try:
        last_spec_num = await asyncio.to_thread(_ms_get_last_spectrum_number, server)
        last_spec = str(last_spec_num)
    except Exception as e:  # noqa: BLE001
        print(f"[MSLOG] 获取最新谱图号失败(/api/ms-log): {e}")

    log_path = _append_ms_log(
        server=server,
        raw_path=raw_path,
        ms_end_time=ms_end_time,
        target_pressure=ctx.current_target,
        voltage=ctx.current_voltage,
        actual_pressure=ctx.current_filtered,
        mfc_opening=ctx.current_flow_cmd,
        last_spectrum_number=last_spec,
    )

    return {
        "ok": True,
        "ms_end_time": ms_end_time,
        "log_path": log_path,
        "target_pressure": ctx.current_target,
        "voltage": ctx.current_voltage,
    }


@app.post("/api/ms-fs-roots")
async def api_ms_fs_roots(body: MsFsRootsBody):
    server = (body.server or ctx.ms_server or "").strip()
    if not server:
        raise RuntimeError("MS 服务器地址不能为空")
    host = server
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]
    base_url = f"http://{host}:8899"
    resp = await asyncio.to_thread(_ms_http_post_json, base_url, "/api/fs/roots", {})
    return resp


@app.post("/api/ms-fs-list")
async def api_ms_fs_list(body: MsFsListBody):
    server = (body.server or ctx.ms_server or "").strip()
    if not server:
        raise RuntimeError("MS 服务器地址不能为空")
    path = (body.path or "").strip()
    if not path:
        raise RuntimeError("路径不能为空")
    host = server
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]
    base_url = f"http://{host}:8899"
    resp = await asyncio.to_thread(_ms_http_post_json, base_url, "/api/fs/list", {"path": path})
    return resp


@app.post("/api/ms-open")
async def api_ms_open(body: MsLogBody):
    """显式打开远程 RAW 文件，并保存当前服务器与路径。

    前端可根据返回的 Open 结果进行校验/打印。
    """
    server = (body.server or "").strip()
    raw_path = (body.raw_path or "").strip()
    if not server:
        raise RuntimeError("MS 服务器地址不能为空")
    if not raw_path:
        raise RuntimeError("raw 文件路径不能为空")

    host = server
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]
    base_url = f"http://{host}:8899"

    open_payload = {"argsNames": [], "args": [raw_path]}
    open_res = _ms_http_post(base_url, "Open", open_payload)

    # 根据返回值判断是否成功，但无论如何都将结果返回前端
    ok = open_res.get("DataType") == 2
    if ok:
        ctx.ms_server = server
        ctx.ms_raw_path = raw_path
        ctx.ms_open = True

    return {"ok": ok, "open_result": open_res}


@app.post("/api/ms-close")
async def api_ms_close():
    """关闭远程 RAW 文件（若有），并清除本地状态。"""
    if ctx.ms_server:
        try:
            host = ctx.ms_server
            if host.startswith("http://"):
                host = host[len("http://") :]
            if host.startswith("https://"):
                host = host[len("https://") :]
            base_url = f"http://{host}:8899"
            payload: Dict[str, Any] = {"argsNames": [], "args": []}
            _ = await asyncio.to_thread(_ms_http_post, base_url, "Close", payload)
        except Exception as e:  # noqa: BLE001
            print(f"[MS] 远程关闭文件失败: {e}")
    ctx.ms_server = None
    ctx.ms_raw_path = None
    ctx.ms_open = False
    return {"ok": True}


@app.post("/api/ms-refresh")
async def api_ms_refresh():
    """对当前已打开的 RAW 文件执行 RefreshViewOfFile 并返回最新结束时间。

    前提：必须已经通过 /api/ms-open 打开远程谱图。
    该接口会在 MSHTTPFastAPI 上调用 RefreshViewOfFile + GetEndTime，
    用于在采集中更新总时间长度，供前端决定是否需要刷新所有 EIC。
    """
    if not (ctx.ms_server and ctx.ms_raw_path and ctx.ms_open):
        return {"ok": False, "error": "尚未通过 /api/ms-open 打开远程谱图"}

    try:
        ms_end_time = await asyncio.to_thread(
            _ms_refresh_and_get_end_time_no_open, ctx.ms_server, ctx.ms_raw_path
        )
        return {"ok": True, "ms_end_time": ms_end_time}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"调用 RefreshViewOfFile 失败: {e}"}


@app.post("/api/ms-chro")
async def api_ms_chro(body: MsChroBody):
    """根据已打开的远程 RAW 文件，获取指定 m/z 范围的 EIC（强度-时间曲线）。

    步骤：
    1. 先在 MSHTTPFastAPI 上调用 checkMassRangeValidate 校验质荷比范围是否合法；
       若不合法，则直接返回错误信息；
    2. 合法时，调用 GetChroData(MassRange1=mass_range)，获取时间与强度数组；
    3. 将时间与强度数据返回给前端用于绘图。

    仅在已经通过 /api/ms-open 打开远程谱图后才可调用本接口。
    """
    if not (ctx.ms_server and ctx.ms_raw_path and ctx.ms_open):
        return {"ok": False, "error": "尚未通过 /api/ms-open 打开远程谱图"}

    mass_range = body.mass_range.strip()
    if not mass_range:
        return {"ok": False, "error": "质荷比范围不能为空"}

    # 解析服务器地址，复用与其它 MS 接口相同的规则
    host = ctx.ms_server.strip()
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]
    base_url = f"http://{host}:8899"

    try:
        # 1) 远程校验 m/z 范围是否合法
        check_payload: Dict[str, Any] = {"argsNames": [], "args": [mass_range]}
        check_res = await asyncio.to_thread(_ms_http_post, base_url, "checkMassRangeValidate", check_payload)
        if check_res.get("DataType") != 2:
            return {
                "ok": False,
                "error": f"远程 checkMassRangeValidate 调用失败: {check_res.get('error')}",
            }
        lst = check_res.get("res") or []
        if not lst:
            return {"ok": False, "error": "checkMassRangeValidate 返回空结果"}

        # MSHTTPFastAPI 中对返回值的包装方式是：
        #   generate_answer(origin_call, res)
        # 若 res 是 (bool, str) 元组，则最终 res 字段为 [bool, str]
        # 因此这里需要兼容两种可能：
        #   1) [ [bool, str] ]
        #   2) [bool, str]
        first = lst[0]
        if isinstance(first, (list, tuple)) and len(first) == 2:
            valid_flag, msg = first
        elif len(lst) >= 2:
            valid_flag, msg = lst[0], lst[1]
        else:
            return {"ok": False, "error": f"checkMassRangeValidate 返回格式不正确: {lst}"}
        if not bool(valid_flag):
            return {"ok": False, "valid": False, "message": msg}

        # 2) 合法时调用 GetChroData(MassRange1=mass_range)
        chro_payload: Dict[str, Any] = {"argsNames": ["MassRange1"], "args": [mass_range]}
        chro_res = await asyncio.to_thread(_ms_http_post, base_url, "GetChroData", chro_payload)
        if chro_res.get("DataType") != 2:
            return {
                "ok": False,
                "error": f"远程 GetChroData 调用失败: {chro_res.get('error')}",
            }
        rlist = chro_res.get("res") or []
        if len(rlist) < 2:
            return {"ok": False, "error": "GetChroData 返回结果格式不正确"}

        time_list = rlist[0]
        intensity_list = rlist[1]

        return {
            "ok": True,
            "valid": True,
            "mass_range": mass_range,
            "time": time_list,
            "intensity": intensity_list,
        }
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"调用 MSHTTPFastAPI 出错: {e}"}


@app.post("/api/ms-chro-lite")
async def api_ms_chro_lite(body: MsChroBody):
    if not (ctx.ms_server and ctx.ms_raw_path and ctx.ms_open):
        return {"ok": False, "error": "尚未通过 /api/ms-open 打开远程谱图"}

    mass_range = body.mass_range.strip()
    if not mass_range:
        return {"ok": False, "error": "质荷比范围不能为空"}

    host = ctx.ms_server.strip()
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]
    base_url = f"http://{host}:8899"

    try:
        payload = {
            "mass_range": mass_range,
            "start_time": body.start_time if body.start_time is not None else 0,
            "end_time": body.end_time if body.end_time is not None else 0,
        }
        resp = await asyncio.to_thread(_ms_http_post_json, base_url, "/api/ms-chro-lite", payload)
        return resp
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"调用 MSHTTPFastAPI 出错: {e}"}


@app.post("/api/open-mfc")
async def api_open_mfc(body: ComConfigBody):
    # 新架构中 MFC 由 STM32 控制，此接口保留占位但不再使用。
    return {"ok": False, "msg": "MFC control is now handled by STM32; no separate MFC port."}


@app.post("/api/close-stm32")
async def api_close_stm32():
    ctx.close_stm32()
    return {"ok": True}


@app.post("/api/close-mfc")
async def api_close_mfc():
    # 无实际操作，仅保持兼容
    return {"ok": True, "msg": "No-op: MFC is controlled by STM32."}


@app.post("/api/start")
async def api_start():
    # 切换 STM32 至控制模式: start_pid = 1
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(1)
    except Exception as e:
        print(f"[CTRL] 写入 start_pid 失败: {e}")
        return {"ok": False, "error": str(e)}
    ctx.start_control()
    return {"ok": True}


@app.post("/api/stop")
async def api_stop():
    # 切换 STM32 至仅检测模式: start_pid = 0（并由固件侧关闭 MFC）
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(0)
    except Exception as e:
        print(f"[CTRL] 写入 start_pid 失败: {e}")
        return {"ok": False, "error": str(e)}
    ctx.stop_control()
    return {"ok": True}


@app.post("/api/save-data")
async def api_save_data():
    """前端点击“保存数据”时调用：将当前缓冲区一次性写入文件。"""
    ctx.save_snapshot()
    return {"ok": True}


@app.post("/api/save-current-data")
async def api_save_current_data():
    """前端点击“保存电流数据”时调用：

    将当前电流历史窗口内的数据一次性写入 CSV 文件，并附带当前连接的远程质谱文件名（若存在）。
    """

    if not ctx._current_history:
        print("[CURRENT] 当前电流历史缓冲区为空，未生成文件")
        return {"ok": False, "empty": True}

    os.makedirs("data", exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    raw_name = os.path.basename(ctx.ms_raw_path) if ctx.ms_raw_path else ""
    filename = os.path.join("data", f"current_{ts}.csv")
    try:
        with open(filename, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow([
                "timestamp",
                "current_A",
                "ms_raw_name",
            ])
            for item in ctx._current_history:
                t, cur_a = item
                ts_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))
                writer.writerow([ts_str, cur_a, raw_name])
        print(f"[CURRENT] 已将 {len(ctx._current_history)} 条电流数据保存到 {filename}")
        return {"ok": True, "path": filename, "ms_raw_name": raw_name}
    except Exception as e:  # noqa: BLE001
        print(f"[CURRENT] 保存电流数据失败: {e}")
        return {"ok": False, "error": str(e)}


@app.post("/api/monitor-on")
async def api_monitor_on():
    """启用仅监测模式：对应 STM32 中 start_pid = 0。"""
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(0)
    except Exception as e:
        print(f"STM32 写入 start_pid 失败: {e}")
        return {"ok": False, "error": f"Failed to write start_pid to STM32: {e}"}
    ctx.current_start_pid = 0
    return {"monitor_only": True}


@app.post("/api/monitor-off")
async def api_monitor_off():
    """关闭仅监测模式：恢复控制模式，对应 STM32 中 start_pid = 1。"""
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(1)
    except Exception as e:
        print(f"STM32 写入 start_pid 失败: {e}")
        return {"ok": False, "error": f"Failed to write start_pid to STM32: {e}"}
    ctx.current_start_pid = 1
    return {"monitor_only": False}


@app.websocket("/ws/pressure")
async def ws_pressure(ws: WebSocket):
    await ws.accept()
    await ctx.register_ws(ws)
    try:
        while True:
            # 保持连接，不强制要求客户端发送内容
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await ctx.unregister_ws(ws)
