"""污染物检测引擎（第一版：最近 1 分钟 EIC 最大值超阈值 + 冷却，结构预留后续复杂条件）。

设计说明：
- 规则是纯数据（tools/detections.json），VSCode 可直接编辑；
- 判定逻辑集中在 Detector 中，条件可在此逐步扩展（持续时长/基线/时间窗等）；
- 检测器只做判断，不负责网络/串口/推送，方便单独测试。
"""

import json
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


DEFAULT_RULES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "detections.json")


@dataclass
class DetectionRule:
    mass_range: str
    threshold: float
    name: str = ""  # 可留空；留空时以 mass_range 作为标识
    enabled: bool = True
    cooldown_s: float = 60.0
    # ---- 预留扩展字段（第一版未使用，后续加条件时启用）----
    min_duration_s: Optional[float] = None   # 持续超过阈值 N 秒才算命中
    baseline_ratio: Optional[float] = None   # 相对基线倍数
    rt_window: Optional[List[float]] = None  # 保留时间窗口 [start, end] (min)
    extra: Dict[str, Any] = field(default_factory=dict)  # 其它自定义字段

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "DetectionRule":
        return cls(
            name=str(d.get("name", "")).strip(),
            mass_range=str(d.get("mass_range", "")).strip(),
            threshold=float(d.get("threshold", 0.0)),
            enabled=bool(d.get("enabled", True)),
            cooldown_s=float(d.get("cooldown_s", 60.0)),
            min_duration_s=d.get("min_duration_s"),
            baseline_ratio=d.get("baseline_ratio"),
            rt_window=d.get("rt_window"),
            extra=dict(d.get("extra") or {}),
        )

    @property
    def key(self) -> str:
        """规则唯一标识：优先名称，名称为空则用 m/z 范围。"""
        return self.name or self.mass_range

    @property
    def display_name(self) -> str:
        """展示名：名称或 m/z 范围。"""
        return self.name or f"m/z {self.mass_range}"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "key": self.key,
            "display_name": self.display_name,
            "name": self.name,
            "mass_range": self.mass_range,
            "threshold": self.threshold,
            "enabled": self.enabled,
            "cooldown_s": self.cooldown_s,
            "min_duration_s": self.min_duration_s,
            "baseline_ratio": self.baseline_ratio,
            "rt_window": self.rt_window,
            "extra": self.extra,
        }


class Detector:
    """检测器：持有规则与运行状态，喂入 EIC 数据，输出命中事件。"""

    def __init__(self, window_s: float = 60.0):
        self._window_s = max(1.0, window_s)
        self._rules: Dict[str, DetectionRule] = {}
        self._lock = threading.RLock()
        # 每条规则的运行状态
        self._recent: Dict[str, List[Tuple[float, float]]] = {}  # (time, intensity)
        self._last_trigger_ts: Dict[str, float] = {}
        self._above_count: Dict[str, int] = {}

    # ---------------- 规则管理 ----------------
    def load_from_file(self, path: str = DEFAULT_RULES_PATH) -> int:
        """从 JSON 文件加载规则，返回规则条数。"""
        rules: Dict[str, DetectionRule] = {}
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                data = []
            items = data if isinstance(data, list) else data.get("rules", [])
            for item in items:
                try:
                    r = DetectionRule.from_dict(item)
                    if r.mass_range:
                        rules[r.key] = r
                except Exception:
                    continue
        with self._lock:
            self._rules = rules
            for key in list(self._recent):
                if key not in rules:
                    del self._recent[key]
            for key in list(self._last_trigger_ts):
                if key not in rules:
                    del self._last_trigger_ts[key]
            for key in list(self._above_count):
                if key not in rules:
                    del self._above_count[key]
        return len(rules)

    def get_rules(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [r.to_dict() for r in self._rules.values()]

    def upsert_rule(self, rule_dict: Dict[str, Any]) -> None:
        r = DetectionRule.from_dict(rule_dict)
        if not r.mass_range:
            raise ValueError("规则缺少 mass_range")
        if r.threshold < 0:
            raise ValueError("threshold 不能为负")
        with self._lock:
            self._rules[r.key] = r

    def delete_rule(self, key: str) -> bool:
        with self._lock:
            existed = self._rules.pop(key, None) is not None
        return existed

    def save_to_file(self, path: str = DEFAULT_RULES_PATH) -> None:
        with self._lock:
            data = {"rules": [r.to_dict() for r in self._rules.values()]}
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)

    # ---------------- 判定 ----------------
    def evaluate(self, mass_range: str, intensity_list: List[float], now_ts: float) -> List[Dict[str, Any]]:
        """喂入某 m/z 范围的 EIC 强度数组，返回触发的检测事件列表。"""
        events: List[Dict[str, Any]] = []
        if not intensity_list:
            return events
        # 本次拉取段（最近约 1 分钟）内的峰值
        segment_peak = max(float(v) for v in intensity_list)
        cutoff = now_ts - self._window_s
        with self._lock:
            for rule in self._rules.values():
                if not rule.enabled or rule.mass_range != mass_range:
                    continue
                key = rule.key
                pts = self._recent.setdefault(key, [])
                pts.append((now_ts, segment_peak))
                self._recent[key] = [p for p in pts if p[0] >= cutoff]
                if not self._conditions_met(rule):
                    continue
                last = self._last_trigger_ts.get(key, 0.0)
                if now_ts - last >= rule.cooldown_s:
                    self._last_trigger_ts[key] = now_ts
                    peak = max(p[1] for p in self._recent[key])
                    events.append({
                        "type": "detection",
                        "key": key,
                        "name": rule.display_name,
                        "mass_range": rule.mass_range,
                        "intensity": peak,
                        "threshold": rule.threshold,
                        "time": now_ts,
                    })
        return events

    def _conditions_met(self, rule: DetectionRule) -> bool:
        """命中条件链。

        第一版：最近 1 分钟窗口内最大强度 > 阈值。
        后续扩展点：在这里按 rule 的 min_duration_s / baseline_ratio / rt_window 等字段追加条件。
        """
        pts = self._recent.get(rule.key, [])
        if not pts:
            return False
        peak = max(p[1] for p in pts)
        if peak <= rule.threshold:
            return False
        # ---- 预留：持续超阈值 N 秒 ----
        if rule.min_duration_s:
            if self._above_count.get(rule.key, 0) < int(rule.min_duration_s):
                self._above_count[rule.key] = self._above_count.get(rule.key, 0) + 1
                return False
        else:
            self._above_count[rule.key] = self._above_count.get(rule.key, 0) + 1
        # ---- 预留：基线倍数 / 时间窗（后续实现）----
        return True
