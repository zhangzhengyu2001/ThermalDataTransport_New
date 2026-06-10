#!/usr/bin/env python3
"""
logger_config.py —— 统一日志配置模块

为 backend_server.py 及所有子模块（modbus_host, digital_power 等）提供：
- 同时输出到控制台和文件的日志记录
- 自动按天轮转，保留最近 30 天
- 统一的日志格式：[时间] [级别] [模块] 消息
- 便捷的函数 get_logger(name) 获取模块专用 logger
"""

import logging
import os
import sys
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler

# 日志目录
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")

# 全局日志格式
_LOG_FORMAT = logging.Formatter(
    fmt="%(asctime)s | %(levelname)-7s | %(name)-20s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

# 简化版格式（仅用于控制台，省略模块名过长时）
_CONSOLE_FORMAT = logging.Formatter(
    fmt="%(asctime)s | %(levelname)-7s | %(name)-18s | %(message)s",
    datefmt="%H:%M:%S",
)

# 全局日志级别（可通过环境变量 LOG_LEVEL 覆盖，默认 DEBUG）
_DEFAULT_LEVEL = getattr(logging, os.environ.get("LOG_LEVEL", "DEBUG").upper(), logging.DEBUG)

# 是否已初始化过根配置
_initialized = False


def init_logging(log_dir: str | None = None, level: int = _DEFAULT_LEVEL) -> None:
    """初始化全局日志系统（幂等，多次调用安全）。

    参数：
        log_dir: 日志目录路径，None 则使用默认 tools/../logs
        level: 根日志级别
    """
    global _initialized
    if _initialized:
        return

    _dir = log_dir or _LOG_DIR
    os.makedirs(_dir, exist_ok=True)

    root = logging.getLogger()
    root.setLevel(level)

    # ---- 控制台 handler ----
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(_CONSOLE_FORMAT)
    root.addHandler(console)

    # ---- 主日志文件 handler（每日轮转，保留 30 天）----
    main_log = os.path.join(_dir, "backend.log")
    file_handler = TimedRotatingFileHandler(
        filename=main_log,
        when="midnight",
        interval=1,
        backupCount=30,
        encoding="utf-8",
    )
    file_handler.setLevel(logging.DEBUG)  # 文件始终记录 DEBUG
    file_handler.setFormatter(_LOG_FORMAT)
    root.addHandler(file_handler)

    # ---- 错误单独文件 ----
    err_log = os.path.join(_dir, "error.log")
    err_handler = TimedRotatingFileHandler(
        filename=err_log,
        when="midnight",
        interval=1,
        backupCount=30,
        encoding="utf-8",
    )
    err_handler.setLevel(logging.WARNING)
    err_handler.setFormatter(_LOG_FORMAT)
    root.addHandler(err_handler)

    _initialized = True

    # 写一条启动标记
    root.info("=" * 60)
    root.info("日志系统初始化完成 | 目录: %s | 级别: %s", _dir, logging.getLevelName(level))
    root.info("=" * 60)


def get_logger(name: str) -> logging.Logger:
    """获取指定名称的 logger。

    用法：
        from tools.logger_config import get_logger
        logger = get_logger(__name__)
        logger.info("串口已打开: %s", port)
    """
    # 确保已初始化（即使调用方忘记显式 init）
    if not _initialized:
        init_logging()
    return logging.getLogger(name)


# ---- 便捷函数：用于非 logger 场景快速打点 ----
_quick_logger = None


def _get_quick():
    global _quick_logger
    if _quick_logger is None:
        _quick_logger = get_logger("quick")
    return _quick_logger


def log_info(fmt: str, *args) -> None:
    _get_quick().info(fmt, *args)


def log_warning(fmt: str, *args) -> None:
    _get_quick().warning(fmt, *args)


def log_error(fmt: str, *args) -> None:
    _get_quick().error(fmt, *args)


def log_debug(fmt: str, *args) -> None:
    _get_quick().debug(fmt, *args)
