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

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from tools.modbus_host import ModbusPressureClient
from tools.logger_config import get_logger, init_logging

# ---- 日志系统初始化 ----
init_logging()
logger = get_logger(__name__)

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
            logger.warning("加载配置失败，使用默认值: %s", e)
    else:
        logger.info("未找到配置文件 %s，使用默认配置", _CONFIG_PATH)
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
    actual_pressure: float,
    mfc_opening: float,
    last_spectrum_number: str,
) -> str:
    """将当前气压、实际气压、MFC 开度与质谱信息写入日志文件，返回日志文件路径。

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
                "actual_pressure",
                "mfc_opening",
                "last_spectrum_number",
                "ms_end_time",
            ])
        writer.writerow(row)

    return log_path


def _ms_log_current_if_open() -> str:
    """记录当前气压状态；若远程 raw 已打开，则先刷新质谱并写入最新时间。"""
    server = ctx.ms_server or ""
    raw_path = ctx.ms_raw_path or ""
    ms_end_time = ""
    last_spec = ""
    if ctx.ms_server and ctx.ms_raw_path and ctx.ms_open:
        try:
            ms_end_time = _ms_refresh_and_get_end_time_no_open(ctx.ms_server, ctx.ms_raw_path)
        except Exception as e:  # noqa: BLE001
            logger.warning("刷新质谱失败: %s", e)
        try:
            last_spec_num = _ms_get_last_spectrum_number(ctx.ms_server)
            last_spec = str(last_spec_num)
        except Exception as e:  # noqa: BLE001
            logger.warning("获取最新谱图号失败: %s", e)
    return _append_ms_log(
        server=server,
        raw_path=raw_path,
        ms_end_time=ms_end_time,
        target_pressure=ctx.current_target,
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

        # 串口与线程资源
        self._modbus_cli: Optional[ModbusPressureClient] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

        # 质谱远程文件状态
        self.ms_server: Optional[str] = None
        self.ms_raw_path: Optional[str] = None
        self.ms_open: bool = False
        # 质谱数据保存文件（打开文件时创建，关闭时重置）
        self._ms_curve_file: Optional[str] = None
        self._ms_spectrum_file: Optional[str] = None
        self._ms_save_lock = threading.Lock()

        # 富集检测模式：设定富集目标气压与时长，结束后自动将目标气压设为 100000 Pa
        self.enrichment_target: float = 0.0
        self.enrichment_duration: float = 0.0  # 秒
        self.enrichment_active: bool = False
        self.enrichment_start_ts: float = 0.0
        self._enrichment_stop = threading.Event()
        self._enrichment_thread = threading.Thread(target=self._enrichment_loop, daemon=True)
        self._enrichment_thread.start()

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
            logger.info("STM32 串口 %s 已打开，跳过重复打开", self.stm32_port)
            return
        logger.info("正在打开 STM32 串口: port=%s, baud=%d, addr=0x%02X",
                     self.stm32_port, self.stm32_baud, self.stm32_addr)
        self._modbus_cli = ModbusPressureClient(
            port=self.stm32_port,
            baudrate=self.stm32_baud,
            timeout=0.5,
            device_addr=self.stm32_addr,
        )
        self._modbus_cli.open()
        logger.info("STM32 串口 %s 打开成功", self.stm32_port)

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

    # ---- 富集检测模式 ----
    def start_enrichment(self, target: float, duration: float):
        """一键开启富集检测模式：写入富集目标气压并开始计时。"""
        if self._modbus_cli is None:
            raise RuntimeError("STM32 串口未打开")
        if target <= 0:
            raise ValueError("富集目标气压必须 > 0")
        if duration <= 0:
            raise ValueError("富集时长必须 > 0")
        self._modbus_cli.write_target_pressure(target)
        self.current_target = target
        self.enrichment_target = target
        self.enrichment_duration = duration
        self.enrichment_active = True
        self.enrichment_start_ts = time.time()
        logger.info("富集检测模式已开启: 目标气压=%.1f Pa, 时长=%.1f s", target, duration)

    def stop_enrichment(self):
        """手动取消富集检测模式（不改变当前目标气压）。"""
        if self.enrichment_active:
            logger.info("富集检测模式已手动取消")
        self.enrichment_active = False

    def enrichment_remaining(self) -> float:
        """返回富集剩余秒数（未开启时为 0）。"""
        if not self.enrichment_active:
            return 0.0
        return max(0.0, self.enrichment_duration - (time.time() - self.enrichment_start_ts))

    def _enrichment_loop(self):
        """后台线程：富集计时结束后自动将目标气压设置为 100000 Pa。"""
        fail_count = 0
        while not self._enrichment_stop.is_set():
            if not self.enrichment_active:
                fail_count = 0
                time.sleep(0.5)
                continue

            remaining = self.enrichment_remaining()
            if remaining > 0:
                time.sleep(min(0.2, remaining))
                continue

            try:
                self._modbus_cli.write_target_pressure(100000.0)
                self.current_target = 100000.0
                self.enrichment_active = False
                fail_count = 0
                logger.info("富集检测模式结束，目标气压已自动设置为 100000 Pa")
                # 自动记录一条日志（若远程 raw 已打开则会刷新质谱）
                try:
                    asyncio.run(_ms_log_current_if_open())
                except Exception as e:  # noqa: BLE001
                    logger.warning("记录富集结束日志失败: %s", e)
            except Exception as e:  # noqa: BLE001
                fail_count += 1
                logger.error("富集结束后设置目标气压失败(第 %d 次): %s", fail_count, e)
                if fail_count >= 5:
                    self.enrichment_active = False
                    logger.error("富集结束写目标气压连续失败 %d 次，已停止富集模式，请手动设置目标气压", fail_count)
                else:
                    time.sleep(2.0)

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
            logger.info("当前历史缓冲区为空，未生成文件")
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
            logger.info("已将 %d 条数据保存到 %s", len(self._history), filename)
        except Exception as e:
            logger.error("保存快照失败: %s", e)

    def _loop(self):
        logger.info("控制循环已启动 (period=%.2fs, window=%.0fs)", self.period, self.history_window)
        last_health_log = time.time()
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

                # 每 10 秒输出一次健康状态摘要
                now = time.time()
                if now - last_health_log > 10.0:
                    logger.debug("控制循环健康: filtered=%.1f Pa, air=%d Pa, target=%.0f Pa, "
                                 "flow=%.4f, pid=%d, 连续错误=%d",
                                 filtered, air, target, flow_cmd, start_pid,
                                 self._modbus_cli.consecutive_errors)
                    last_health_log = now

            except Exception as e:
                cons_err = self._modbus_cli.consecutive_errors if self._modbus_cli else -1
                logger.error("读取气压失败: %s | 连续错误=%d", e, cons_err)
                # 连续错误过多时发出警告
                if cons_err >= 3:
                    logger.warning("STM32 通信连续失败 %d 次，请检查串口连接和 STM32 状态！", cons_err)
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

        所有监控/控制线程只负责更新 ctx 内部状态，
        不直接调用 _broadcast，从而避免多线程重复或竞争广播。
        """

        while not self._broadcast_stop.is_set():
            t0 = time.time()
            try:
                asyncio.run(self._broadcast())
            except Exception as e:  # noqa: BLE001
                logger.error("广播线程发送数据失败: %s", e)

            elapsed = time.time() - t0
            # 使用 period 作为默认广播间隔；若非常小则兜底为 10ms
            sleep_t = self.period - elapsed
            if sleep_t < 0.01:
                sleep_t = 0.01
            time.sleep(sleep_t)

    async def register_ws(self, ws: WebSocket):
        async with self._ws_lock:
            self._ws_clients.append(ws)
            logger.info("WebSocket 客户端已连接 (当前共 %d 个)", len(self._ws_clients))

    async def unregister_ws(self, ws: WebSocket):
        async with self._ws_lock:
            if ws in self._ws_clients:
                self._ws_clients.remove(ws)
                logger.info("WebSocket 客户端已断开 (当前共 %d 个)", len(self._ws_clients))

    async def _broadcast(self):
        data = {
            "timestamp": time.time(),
            "filtered_pressure": self.current_filtered,
            "air_pressure": self.current_air,
            "target_pressure": self.current_target,
            "flow_cmd": self.current_flow_cmd,
            "period": self.period,
            "mode": "control" if self.current_start_pid else "monitor",
            "enrichment": {
                "active": self.enrichment_active,
                "target": self.enrichment_target,
                "duration": self.enrichment_duration,
                "remaining": self.enrichment_remaining(),
            },
            "comm": {
                "open": bool(self._modbus_cli and self._modbus_cli._ser and self._modbus_cli._ser.is_open),
                "consecutive_errors": self._modbus_cli.consecutive_errors if self._modbus_cli else -1,
            },
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
            if dead:
                logger.info("已移除 %d 个断开的 WebSocket 客户端 (剩余 %d)",
                           len(dead), len(self._ws_clients))


ctx = ControlContext()

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---- 前端静态页面 (tools/web) ----
_WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
if os.path.isdir(_WEB_DIR):
    app.mount("/web", StaticFiles(directory=_WEB_DIR, html=True), name="web")
else:
    logger.warning("未找到前端静态目录 %s，/web 路由不可用", _WEB_DIR)


@app.get("/")
async def root():
    """根路径跳转到前端主页面。"""
    return RedirectResponse("/web/index.html")


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


class EnrichmentBody(BaseModel):
    target: float  # 富集目标气压 (Pa)
    duration: float  # 富集时长 (秒)


class ComConfigBody(BaseModel):
    port: str
    baudrate: int
    addr: Optional[int] = None


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


class MsCurvesBody(BaseModel):
    time: List[float] = []  # 时间点 (min)
    tic: List[float] = []  # TIC 强度
    eics: List[Dict[str, Any]] = []  # [{"mass_range": "...", "intensity": [...]}]


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
        "enrichment": {
            "active": ctx.enrichment_active,
            "target": ctx.enrichment_target,
            "duration": ctx.enrichment_duration,
            "remaining": ctx.enrichment_remaining(),
        },
        "comm": {
            "open": bool(ctx._modbus_cli and ctx._modbus_cli._ser and ctx._modbus_cli._ser.is_open),
            "consecutive_errors": ctx._modbus_cli.consecutive_errors if ctx._modbus_cli else -1,
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
        logger.error("写入目标气压失败: %s", e)
        return {"error": str(e)}
    ctx.current_target = t
    # 自动记录一条日志（若远程 raw 已打开则会刷新质谱）
    try:
        await asyncio.to_thread(_ms_log_current_if_open)
    except Exception as e:  # noqa: BLE001
        logger.warning("记录日志失败(设置目标气压): %s", e)
    return {"target": ctx.current_target}


@app.post("/api/enrichment/start")
async def api_enrichment_start(body: EnrichmentBody):
    """一键开启富集检测模式：设置富集目标气压与时长并开始计时。

    富集结束后会自动将目标气压设置为 100000 Pa。
    """
    ctx.start_enrichment(target=float(body.target), duration=float(body.duration))
    # 自动记录一条日志（若远程 raw 已打开则会刷新质谱）
    try:
        await asyncio.to_thread(_ms_log_current_if_open)
    except Exception as e:  # noqa: BLE001
        logger.warning("记录日志失败(开启富集): %s", e)
    return {
        "ok": True,
        "enrichment": {
            "active": ctx.enrichment_active,
            "target": ctx.enrichment_target,
            "duration": ctx.enrichment_duration,
            "remaining": ctx.enrichment_remaining(),
        },
    }


@app.post("/api/enrichment/stop")
async def api_enrichment_stop():
    """手动取消富集检测模式（不改变当前目标气压）。"""
    ctx.stop_enrichment()
    return {"ok": True, "enrichment": {"active": False}}


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
    return {"history_window": ctx.history_window}


@app.post("/api/open-stm32")
async def api_open_stm32(body: ComConfigBody):
    logger.info("API: 打开 STM32 串口 port=%s baud=%d addr=%s", body.port, body.baudrate, body.addr)
    ctx.stm32_port = body.port
    ctx.stm32_baud = body.baudrate
    if body.addr is not None:
        ctx.stm32_addr = body.addr
    ctx.open_stm32()
    return {"ok": True, "port": ctx.stm32_port, "baud": ctx.stm32_baud, "addr": ctx.stm32_addr}


@app.post("/api/ms-log")
async def api_ms_log(body: MsLogBody):
    """远程调用 MSHTTPFastAPI 刷新指定 raw 文件并写入日志。

    步骤：
    1. 在 MS 服务器上 Open(raw_path)
    2. 调用 RefreshViewOfFile
    3. 调用 GetEndTime 获取质谱最新时间
    4. 将当前目标气压、实际气压、raw 文件名和质谱时间写入本地日志
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
        logger.warning("获取最新谱图号失败(/api/ms-log): %s", e)

    log_path = _append_ms_log(
        server=server,
        raw_path=raw_path,
        ms_end_time=ms_end_time,
        target_pressure=ctx.current_target,
        actual_pressure=ctx.current_filtered,
        mfc_opening=ctx.current_flow_cmd,
        last_spectrum_number=last_spec,
    )

    return {
        "ok": True,
        "ms_end_time": ms_end_time,
        "log_path": log_path,
        "target_pressure": ctx.current_target,
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
        # 为本次打开的 raw 文件建立曲线/谱图保存文件
        os.makedirs("data", exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        stem = os.path.splitext(os.path.basename(raw_path))[0]
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in stem) or "raw"
        ctx._ms_curve_file = os.path.join("data", f"ms_curves_{safe}_{ts}.csv")
        ctx._ms_spectrum_file = os.path.join("data", f"ms_spectra_{safe}_{ts}.csv")

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
            logger.warning("远程关闭文件失败: %s", e)
    ctx.ms_server = None
    ctx.ms_raw_path = None
    ctx.ms_open = False
    ctx._ms_curve_file = None
    ctx._ms_spectrum_file = None
    return {"ok": True}


@app.post("/api/ms/curves-append")
async def api_ms_curves_append(body: MsCurvesBody):
    """方案 B：追加保存增量 TIC/EIC 曲线数据（长表格式 CSV）。"""
    if not ctx.ms_open or not ctx._ms_curve_file:
        raise RuntimeError("尚未打开远程谱图，无法保存曲线数据")

    times = [float(x) for x in body.time]
    if not times:
        return {"ok": True, "saved": 0}

    os.makedirs("data", exist_ok=True)
    is_new = not os.path.exists(ctx._ms_curve_file)
    rows = 0
    with ctx._ms_save_lock:
        with open(ctx._ms_curve_file, "a", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            if is_new:
                writer.writerow(["time_min", "series", "intensity"])
            tic = list(body.tic)
            if len(tic) == len(times):
                for t, v in zip(times, tic):
                    writer.writerow([f"{t:.4f}", "TIC", f"{v}"])
                rows += len(times)
            for e in body.eics:
                label = f"EIC {e.get('mass_range', '')}"
                vals = list(e.get("intensity") or [])
                if len(vals) == len(times):
                    for t, v in zip(times, vals):
                        writer.writerow([f"{t:.4f}", label, f"{v}"])
                    rows += len(times)
    logger.info("已追加 %d 行质谱曲线数据到 %s", rows, ctx._ms_curve_file)
    return {"ok": True, "saved": rows}


@app.post("/api/ms/spectrum-snapshot")
async def api_ms_spectrum_snapshot():
    """方案 C：保存最新一张谱图的完整 m/z-强度数据。"""
    if not ctx.ms_open or not ctx._ms_spectrum_file:
        raise RuntimeError("尚未打开远程谱图，无法保存谱图快照")

    server = ctx.ms_server or ""
    host = server.strip()
    if host.startswith("http://"):
        host = host[len("http://") :]
    if host.startswith("https://"):
        host = host[len("https://") :]
    base_url = f"http://{host}:8899"

    # 1) 获取最新谱图号
    try:
        spec_num = await asyncio.to_thread(_ms_get_last_spectrum_number, server)
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"获取最新谱图号失败: {e}") from e

    # 2) 拉取该谱图的 m/z 与强度数组
    payload: Dict[str, Any] = {"argsNames": [], "args": [spec_num]}
    res = await asyncio.to_thread(_ms_http_post, base_url, "GetMassListFromScanNum", payload)
    if res.get("DataType") != 2:
        raise RuntimeError(f"远程 GetMassListFromScanNum 调用失败: {res.get('error')}")
    rlist = res.get("res") or []
    if len(rlist) < 2:
        raise RuntimeError("GetMassListFromScanNum 返回格式不正确")
    mz = rlist[0]
    sig = rlist[1]
    if not isinstance(mz, list) or not isinstance(sig, list) or len(mz) != len(sig):
        raise RuntimeError("GetMassListFromScanNum 返回的数据长度不一致")

    os.makedirs("data", exist_ok=True)
    is_new = not os.path.exists(ctx._ms_spectrum_file)
    now_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    with ctx._ms_save_lock:
        with open(ctx._ms_spectrum_file, "a", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            if is_new:
                writer.writerow(["recorded_at", "scan_num", "mz", "intensity"])
            for m, s in zip(mz, sig):
                writer.writerow([now_str, spec_num, m, s])
    logger.info("已保存谱图快照 scan=%d (%d 点) 到 %s", spec_num, len(mz), ctx._ms_spectrum_file)
    return {"ok": True, "scan_num": spec_num, "points": len(mz)}


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
    logger.info("API: 关闭 STM32 串口 (port=%s)", ctx.stm32_port)
    ctx.close_stm32()
    return {"ok": True}


@app.post("/api/close-mfc")
async def api_close_mfc():
    # 无实际操作，仅保持兼容
    return {"ok": True, "msg": "No-op: MFC is controlled by STM32."}


@app.post("/api/start")
async def api_start():
    logger.info("API: 开始控制 (start_pid=1)")
    # 切换 STM32 至控制模式: start_pid = 1
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(1)
    except Exception as e:
        logger.error("写入 start_pid=1 失败: %s", e)
        return {"ok": False, "error": str(e)}
    ctx.start_control()
    return {"ok": True}


@app.post("/api/stop")
async def api_stop():
    logger.info("API: 停止控制 (start_pid=0)")
    # 切换 STM32 至仅检测模式: start_pid = 0（并由固件侧关闭 MFC）
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(0)
    except Exception as e:
        logger.error("写入 start_pid=0 失败: %s", e)
        return {"ok": False, "error": str(e)}
    ctx.stop_control()
    return {"ok": True}


@app.post("/api/save-data")
async def api_save_data():
    """前端点击“保存数据”时调用：将当前缓冲区一次性写入文件。"""
    ctx.save_snapshot()
    return {"ok": True}


@app.post("/api/monitor-on")
async def api_monitor_on():
    """启用仅监测模式：对应 STM32 中 start_pid = 0。"""
    if ctx._modbus_cli is None:
        raise RuntimeError("STM32 串口未打开")
    try:
        ctx._modbus_cli.write_start_pid(0)
    except Exception as e:
        logger.error("写入 start_pid=0 失败 (monitor-on): %s", e)
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
        logger.error("写入 start_pid=1 失败 (monitor-off): %s", e)
        return {"ok": False, "error": f"Failed to write start_pid to STM32: {e}"}
    ctx.current_start_pid = 1
    return {"monitor_only": False}


@app.get("/api/health")
async def api_health():
    """设备连接健康检查接口。

    返回 STM32 通信状态等，供前端判断通信是否正常。
    """
    health = {
        "stm32": {
            "port": ctx.stm32_port,
            "open": bool(ctx._modbus_cli and ctx._modbus_cli._ser and ctx._modbus_cli._ser.is_open),
            "healthy": ctx._modbus_cli.is_healthy() if ctx._modbus_cli else False,
            "consecutive_errors": ctx._modbus_cli.consecutive_errors if ctx._modbus_cli else -1,
        },
        "ms": {
            "server": ctx.ms_server,
            "open": ctx.ms_open,
        },
    }
    overall = health["stm32"]["open"]
    logger.debug("健康检查: stm32=%s", health["stm32"]["healthy"])
    return {"ok": overall, "health": health}


@app.get("/api/logs/recent")
async def api_logs_recent(lines: int = 80, level: str = "all"):
    """返回最近日志（读取 logs/backend.log 末尾若干行，可按级别过滤）。"""
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
    log_path = os.path.join(log_dir, "backend.log")
    if not os.path.exists(log_path):
        return {"ok": True, "lines": [], "path": str(log_path)}

    lines = max(1, min(int(lines), 500))
    level_filters = {
        "all": None,
        "warning": ("WARNING", "ERROR", "CRITICAL"),
        "error": ("ERROR", "CRITICAL"),
        "info": ("INFO", "WARNING", "ERROR", "CRITICAL"),
    }
    allowed = level_filters.get(level, None)

    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as fh:
            tail = fh.readlines()
        # 只保留最近的日志（取足够大的尾部再过滤，避免全文件扫描）
        tail = tail[-max(500, lines * 4):]
        if allowed is not None:
            tail = [ln for ln in tail if any(f" | {lv}" in ln for lv in allowed)]
        result = [ln.rstrip("\n") for ln in tail[-lines:]]
        return {"ok": True, "lines": result, "path": str(log_path)}
    except Exception as e:  # noqa: BLE001
        logger.error("读取日志失败: %s", e)
        return {"ok": False, "error": str(e)}


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
