# /// script
# requires-python = ">=3.12"
# dependencies = []
# [tool.orcaslicer.plugin]
# name = "更好的外墙处理"
# description = "外墙立体斜接缝（scarf）与二次整形后处理插件，附带中文设置与说明。"
# author = "OpenAI Codex"
# version = "0.3.0"
# ///
import json
import math
import os
import sys
import time
import tempfile
import argparse
from bisect import bisect_right
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:
    import orca
except ImportError:
    class _ExecutionResult:
        @staticmethod
        def success(message="", data=""):
            return {"status": "success", "message": message, "data": data}

        @staticmethod
        def skipped(message=""):
            return {"status": "skipped", "message": message, "data": ""}

        @staticmethod
        def failure(status, message, data=""):
            return {"status": status, "message": message, "data": data}

    class _PluginResult:
        RecoverableError = "RecoverableError"

    class _Base:
        pass

    class _GCodeBase:
        pass

    class _ScriptBase:
        pass

    class _HostUi:
        @staticmethod
        def show_dialog(**kwargs):
            return None

    class _Host:
        ui = _HostUi()

    class _GCodeModule:
        GCodePluginCapabilityBase = _GCodeBase

    class _ScriptModule:
        ScriptPluginCapabilityBase = _ScriptBase

    class _OrcaFallback:
        ExecutionResult = _ExecutionResult
        PluginResult = _PluginResult
        base = _Base
        gcode = _GCodeModule()
        script = _ScriptModule()
        host = _Host()

        @staticmethod
        def plugin(cls):
            return cls

        @staticmethod
        def register_capability(cls):
            return cls

    orca = _OrcaFallback()


PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(PLUGIN_DIR, "better_outer_wall_processing.json")
LOG_PATH = os.path.join(PLUGIN_DIR, "progress.log")
MANUAL_TEXT = """# 更好的外墙处理使用说明书（内置摘要）

## 功能概述

1. 立体斜接缝（scarf seam，x 倍超分）：外墙起始 `scarf_length_mm` 范围内
   Z 从下一层高度渐升到本层高度、流量从 `primary_seam_min_scale` 渐增；
   外墙走完后沿同一路径在本层高度补一段互补流量的重叠段，上下两个楔形
   拼合成斜接缝。`seam_superres_x` 控制渐变细分倍数，越大越平滑。
2. 二次整形：外墙完成后从偏移起点低流量、低速度重走一遍整形。

## 兼容性

- 支持绝对挤出（M82）与相对挤出（M83），支持 G92 E 重置。
- G2/G3 outer-wall arcs with IJ centers are expanded for processing; only unsupported arc loops are skipped.
- 首层或无法确定层高时自动退化为纯流量渐变（Z 不下探），避免撞热床。
- 插入段前后自动恢复原始进给速度；若外墙结束时已回抽会先回填再重新回抽。

## 回退方式

关闭整个插件；或关闭立体斜接缝（退化为流量渐变）；或把二次整形模式改为
局部窗口；或降低二次整形流量。
"""

OUTER_WALL_MARKERS = (
    "TYPE:OUTER WALL",
    "TYPE:EXTERNAL PERIMETER",
    "FEATURE: OUTER WALL",
    "FEATURE: EXTERNAL PERIMETER",
)
INNER_WALL_MARKERS = (
    "TYPE:INNER WALL",
    "TYPE:INTERNAL PERIMETER",
    "FEATURE: INNER WALL",
    "FEATURE: INTERNAL PERIMETER",
)
# 悬垂墙标记只延续已开始的外墙回路（内墙的悬垂段也用同一标记，
# 不能凭它开启新回路，否则内墙开口悬垂片段会被当成外墙处理）
OVERHANG_WALL_MARKERS = (
    "TYPE:OVERHANG WALL",
    "TYPE:OVERHANG PERIMETER",
    "FEATURE: OVERHANG WALL",
)
LAYER_MARKERS = ("LAYER_CHANGE", "CHANGE_LAYER", "LAYER:")
BOWP_TAG = "BOWP"
DEFAULT_FEED = 1800.0
UNRETRACT_FEED = 1800.0
TRAVEL_GAP_MM = 0.5
LOOP_CLOSE_TOL_MM = 0.25
LOOP_LEADIN_CLOSE_TOL_MM = 0.05
LOOP_LEADIN_MAX_MM = 0.75
MAX_GENERATED_LOOP_SEGMENTS = 2000
DEFAULT_SPIRAL_ANGLE_SPEED_PROFILE = "45:1.0,30:0.8,15:0.4,0:0.33"
DEFAULT_SPIRAL_ANGLE_SPEED_POINTS: Tuple[Tuple[float, float], ...] = (
    (0.0, 0.33),
    (15.0, 0.4),
    (30.0, 0.8),
    (45.0, 1.0),
)


DEFAULT_CONFIG: Dict[str, Any] = {
    "enabled": True,
    "seam_processing_enabled": True,
    "seam_mode": "spiral",
    "seam_superres_x": 4,
    "single_wall_mask_enabled": False,
    "dynamic_superres_enabled": True,
    "spiral_overlap_frac": 0.5,
    "superres_max_x": 12,
    "spiral_flatten_enabled": False,
    "spiral_xy_interp_enabled": True,
    "ironing_trim_enabled": True,
    "script_ironing_enabled": True,
    "script_ironing_flow": 0.10,
    "script_ironing_speed_mm_s": 30.0,
    "path_width_mm": 0.42,
    "seam_comp_length_mm": 2.0,
    "seam_comp_scale": 1.0,
    "purge_retract_mm": 1.0,
    "max_overhang_deg": -90.0,
    "spiral_speed_scale": 0.66,
    "spiral_angle_speed_enabled": False,
    "spiral_angle_speed_profile": DEFAULT_SPIRAL_ANGLE_SPEED_PROFILE,
    "sample_step_mm": 0.2,
    "keep_processed_copy": False,
    "scarf_length_mm": 3.2,
    "min_wall_length_mm": 4.0,
    "wall_length_limit_mm": 1.0,
    "short_wall_fallback_rel": 0.25,
    "primary_flow_scale": 0.96,
    "primary_seam_min_scale": 0.4,
    "secondary_pass_enabled": True,
    "secondary_flow_scale": 0.08,
    "secondary_speed_scale": 1.0,
    "secondary_seam_speed_scale": 0.67,
    "secondary_start_offset_abs_mm": 2.6,
    "secondary_start_offset_rel": 0.25,
    "secondary_mode": "full_loop",
    "secondary_window_mm": 24.0,
    "travel_speed_mm_s": 150.0,
}

CONFIG_FIELDS: List[Dict[str, Any]] = [
    {"key": "enabled", "label": "启用脚本", "kind": "bool"},
    {"key": "seam_processing_enabled", "label": "启用接缝处理", "kind": "bool"},
    {"key": "seam_mode", "label": "接缝模式", "kind": "enum", "options": ["spiral", "scarf", "flat"]},
    {"key": "seam_superres_x", "label": "超分倍数(每层)", "kind": "int"},
    {"key": "single_wall_mask_enabled", "label": "屏蔽单层墙", "kind": "bool"},
    {"key": "dynamic_superres_enabled", "label": "动态超分倍率", "kind": "bool"},
    {"key": "spiral_overlap_frac", "label": "子圈重叠比例(线宽)", "kind": "float"},
    {"key": "superres_max_x", "label": "动态超分上限(圈)", "kind": "int"},
    {"key": "spiral_flatten_enabled", "label": "末圈空转压平", "kind": "bool"},
    {"key": "seam_comp_length_mm", "label": "接缝补偿长度(mm)", "kind": "float"},
    {"key": "seam_comp_scale", "label": "接缝补偿强度", "kind": "float"},
    {"key": "purge_retract_mm", "label": "排压圈回抽量(mm)", "kind": "float"},
    {"key": "max_overhang_deg", "label": "最大悬垂角(度)", "kind": "float"},
    {"key": "spiral_xy_interp_enabled", "label": "亚层级精度优化 XY 插值", "kind": "bool"},
    {"key": "ironing_trim_enabled", "label": "移除覆盖区原生熨烫", "kind": "bool"},
    {"key": "script_ironing_enabled", "label": "脚本熨烫裸露顶面", "kind": "bool"},
    {"key": "script_ironing_flow", "label": "脚本熨烫流量", "kind": "float"},
    {"key": "script_ironing_speed_mm_s", "label": "脚本熨烫速度(mm/s)", "kind": "float"},
    {"key": "path_width_mm", "label": "路径粗细(mm)", "kind": "float"},
    {"key": "spiral_speed_scale", "label": "螺旋速度倍率(1=原速)", "kind": "float"},
    {"key": "spiral_angle_speed_enabled", "label": "启用螺旋小角度降速", "kind": "bool"},
    {"key": "spiral_angle_speed_profile", "label": "角度速度曲线(度:倍率)", "kind": "text"},
    {"key": "sample_step_mm", "label": "采样精度(mm)", "kind": "float"},
    {"key": "keep_processed_copy", "label": "保留处理副本", "kind": "bool"},
    {"key": "scarf_length_mm", "label": "斜拼长度(mm)", "kind": "float"},
    {"key": "min_wall_length_mm", "label": "低于此墙长不斜拼(mm)", "kind": "float"},
    {"key": "wall_length_limit_mm", "label": "极限周长(mm)", "kind": "float"},
    {"key": "short_wall_fallback_rel", "label": "短墙兜底百分比", "kind": "float"},
    {"key": "primary_flow_scale", "label": "外墙主体流量缩放", "kind": "float"},
    {"key": "primary_seam_min_scale", "label": "接缝起点流量缩放", "kind": "float"},
    {"key": "secondary_pass_enabled", "label": "启用二次整形", "kind": "bool"},
    {"key": "secondary_flow_scale", "label": "二次整形流量缩放", "kind": "float"},
    {"key": "secondary_speed_scale", "label": "整形速度倍率(1=原速)", "kind": "float"},
    {"key": "secondary_seam_speed_scale", "label": "整形接缝段速度倍率", "kind": "float"},
    {"key": "travel_speed_mm_s", "label": "空移速度(mm/s)", "kind": "float"},
    {"key": "secondary_start_offset_abs_mm", "label": "起点绝对偏移(mm)", "kind": "float"},
    {"key": "secondary_start_offset_rel", "label": "短外墙相对偏移", "kind": "float"},
    {"key": "secondary_mode", "label": "二次整形模式", "kind": "enum", "options": ["full_loop", "local_window"]},
    {"key": "secondary_window_mm", "label": "局部窗口长度(mm)", "kind": "float"},
]


@dataclass(slots=True)
class GCodeLine:
    raw: str
    command: Optional[str]
    args: Dict[str, float]
    comment: str


@dataclass(slots=True)
class LineInfo:
    index: int
    line: GCodeLine
    kind: str  # move / g92 / m82 / m83 / other
    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float
    de: float
    feed: float = DEFAULT_FEED


@dataclass(slots=True)
class Segment:
    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float
    de: float
    length: float
    line_index: int
    dx: float = field(init=False, repr=False)
    dy: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.dx = self.x1 - self.x0
        self.dy = self.y1 - self.y0


@dataclass(slots=True)
class Loop:
    layer_index: int
    z: float
    segments: List[Segment]
    end_insert_index: int
    has_arc: bool = False
    y_min: float = field(init=False)
    y_max: float = field(init=False)
    cum: List[float] = field(default_factory=list)
    grid: Optional[Dict[Tuple[int, int], List[int]]] = field(default=None, repr=False)
    verts_xy: Optional[List[Tuple[float, float]]] = field(default=None, repr=False)
    normals_xy: Optional[List[Tuple[float, float]]] = field(default=None, repr=False)
    centroid_xy: Optional[Tuple[float, float]] = field(default=None, repr=False)
    offset_cache: Optional[Dict[Tuple[int, int], List[Tuple[float, float]]]] = field(default=None, repr=False)
    scanline_grid: Optional[Dict[int, List[Segment]]] = field(default=None, repr=False)
    scanline_cache: Optional[Dict[float, List[Tuple[float, float]]]] = field(default=None, repr=False)
    nearest_data: Optional[List[Tuple[float, float, float, float, float]]] = field(default=None, repr=False)
    geometry_loop: Optional["Loop"] = field(default=None, init=False, repr=False)
    geometry_resolved: bool = field(default=False, init=False, repr=False)
    seam_leadin: Optional[bool] = field(default=None, init=False, repr=False)

    @property
    def total_length(self) -> float:
        return self.cum[-1] if self.cum else 0.0

    def __post_init__(self) -> None:
        ys = [v for seg in self.segments for v in (seg.y0, seg.y1)]
        self.y_min = min(ys) if ys else 0.0
        self.y_max = max(ys) if ys else 0.0

    def build_cum(self) -> None:
        acc = 0.0
        self.cum = []
        for seg in self.segments:
            acc += seg.length
            self.cum.append(acc)


@dataclass(slots=True)
class Ins:
    kind: str  # comment / travel / extrude / prime
    text: str = ""
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    de: float = 0.0
    f: Optional[float] = None


@dataclass(slots=True)
class EstimateStats:
    base_time_s: float
    final_time_s: float
    added_time_s: float
    base_path_mm: float
    final_path_mm: float
    added_path_mm: float
    base_extrusion_mm: float
    final_extrusion_mm: float
    added_extrusion_mm: float
    arc_loops_skipped: int = 0
    overhang_demoted: int = 0
    ironing_trimmed: int = 0
    script_ironing_lines: int = 0


def empty_stats() -> EstimateStats:
    return EstimateStats(0, 0, 0, 0, 0, 0, 0, 0, 0)


def debug_log(message: str) -> None:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{timestamp}] {message}\n"
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)
    except OSError:
        pass
    try:
        sys.stderr.write(line)
    except Exception:
        pass


def load_config() -> Dict[str, Any]:
    config = dict(DEFAULT_CONFIG)
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                config.update(data)
        except Exception as exc:
            debug_log(f"load_config failed: {exc}")
    return config


def save_config(config: Dict[str, Any]) -> None:
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=2)


def coerce_config_value(kind: str, raw: Any) -> Any:
    if kind == "bool":
        return bool(raw)
    if kind == "int":
        return int(raw)
    if kind == "float":
        return float(raw)
    return str(raw)


def fnum(config: Dict[str, Any], key: str) -> float:
    try:
        return float(config.get(key, DEFAULT_CONFIG[key]))
    except (TypeError, ValueError):
        return float(DEFAULT_CONFIG[key])


def parse_spiral_angle_speed_profile(raw: Any) -> Tuple[Tuple[float, float], ...]:
    """Parse an angle:multiplier profile and return points sorted by angle."""
    if not isinstance(raw, str):
        raise ValueError("angle speed profile must be text")
    points: List[Tuple[float, float]] = []
    seen: set[float] = set()
    for item in raw.split(","):
        item = item.strip()
        if not item or item.count(":") != 1:
            raise ValueError("angle speed profile must use angle:multiplier pairs")
        angle_text, scale_text = item.split(":", 1)
        try:
            angle = float(angle_text.strip())
            scale = float(scale_text.strip())
        except ValueError as exc:
            raise ValueError("angle speed profile contains a non-numeric value") from exc
        if not math.isfinite(angle) or not 0.0 <= angle <= 90.0:
            raise ValueError("angle speed profile angles must be within 0..90 degrees")
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError("angle speed profile multipliers must be positive")
        if angle in seen:
            raise ValueError("angle speed profile contains a duplicate angle")
        seen.add(angle)
        points.append((angle, scale))
    if len(points) < 2:
        raise ValueError("angle speed profile requires at least two points")
    points.sort(key=lambda point: point[0])
    return tuple(points)


def configured_spiral_angle_speed_profile(
    config: Dict[str, Any],
) -> Tuple[Tuple[float, float], ...]:
    try:
        return parse_spiral_angle_speed_profile(
            config.get("spiral_angle_speed_profile", DEFAULT_SPIRAL_ANGLE_SPEED_PROFILE)
        )
    except (TypeError, ValueError):
        return DEFAULT_SPIRAL_ANGLE_SPEED_POINTS


def spiral_angle_speed_multiplier(
    angle_deg: float,
    profile: Sequence[Tuple[float, float]],
) -> float:
    angle = max(0.0, min(90.0, angle_deg))
    if angle <= profile[0][0]:
        return profile[0][1]
    for (angle0, scale0), (angle1, scale1) in zip(profile, profile[1:]):
        if angle <= angle1:
            span = angle1 - angle0
            if span <= 1e-12:
                return scale1
            t = (angle - angle0) / span
            return scale0 + (scale1 - scale0) * t
    return profile[-1][1]


def _cli_angle_speed_profile(raw: str) -> str:
    try:
        parse_spiral_angle_speed_profile(raw)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return raw.strip()


def apply_cli_overrides(config: Dict[str, Any], argv: Sequence[str]) -> Tuple[Dict[str, Any], str]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--secondary-speed-scale", dest="secondary_speed_scale", type=float)
    parser.add_argument("--secondary-seam-speed-scale", dest="secondary_seam_speed_scale", type=float)
    parser.add_argument("--travel-speed", dest="travel_speed_mm_s", type=float)
    parser.add_argument("--secondary-flow", dest="secondary_flow_scale", type=float)
    parser.add_argument("--scarf-length", dest="scarf_length_mm", type=float)
    parser.add_argument("--min-wall-length", dest="min_wall_length_mm", type=float)
    parser.add_argument("--wall-limit", dest="wall_length_limit_mm", type=float)
    parser.add_argument("--short-fallback", dest="short_wall_fallback_rel", type=float)
    parser.add_argument("--primary-flow", dest="primary_flow_scale", type=float)
    parser.add_argument("--seam-min-flow", dest="primary_seam_min_scale", type=float)
    parser.add_argument("--seam-superres", dest="seam_superres_x", type=int)
    parser.add_argument("--enable-single-wall-mask", dest="single_wall_mask_enabled", action="store_true", default=None)
    parser.add_argument("--disable-single-wall-mask", dest="single_wall_mask_enabled", action="store_false", default=None)
    parser.add_argument("--enable-dynamic-superres", dest="dynamic_superres_enabled", action="store_true", default=None)
    parser.add_argument("--disable-dynamic-superres", dest="dynamic_superres_enabled", action="store_false", default=None)
    parser.add_argument("--spiral-overlap", dest="spiral_overlap_frac", type=float)
    parser.add_argument("--superres-max", dest="superres_max_x", type=int)
    parser.add_argument("--seam-comp-length", dest="seam_comp_length_mm", type=float)
    parser.add_argument("--seam-comp-scale", dest="seam_comp_scale", type=float)
    parser.add_argument("--purge-retract", dest="purge_retract_mm", type=float)
    parser.add_argument("--max-overhang", dest="max_overhang_deg", type=float)
    parser.add_argument("--seam-mode", dest="seam_mode", choices=["spiral", "scarf", "flat"])
    parser.add_argument("--enable-flatten", dest="spiral_flatten_enabled", action="store_true", default=None)
    parser.add_argument("--disable-flatten", dest="spiral_flatten_enabled", action="store_false", default=None)
    parser.add_argument("--enable-xy-interp", dest="spiral_xy_interp_enabled", action="store_true", default=None)
    parser.add_argument("--disable-xy-interp", dest="spiral_xy_interp_enabled", action="store_false", default=None)
    parser.add_argument("--enable-ironing-trim", dest="ironing_trim_enabled", action="store_true", default=None)
    parser.add_argument("--disable-ironing-trim", dest="ironing_trim_enabled", action="store_false", default=None)
    parser.add_argument("--enable-script-ironing", dest="script_ironing_enabled", action="store_true", default=None)
    parser.add_argument("--disable-script-ironing", dest="script_ironing_enabled", action="store_false", default=None)
    parser.add_argument("--script-ironing-flow", dest="script_ironing_flow", type=float)
    parser.add_argument("--script-ironing-speed", dest="script_ironing_speed_mm_s", type=float)
    parser.add_argument("--path-width", dest="path_width_mm", type=float)
    parser.add_argument("--spiral-speed-scale", dest="spiral_speed_scale", type=float)
    parser.add_argument(
        "--enable-angle-speed", "--enable-spiral-angle-speed",
        dest="spiral_angle_speed_enabled", action="store_true", default=None,
    )
    parser.add_argument(
        "--disable-angle-speed", "--disable-spiral-angle-speed",
        dest="spiral_angle_speed_enabled", action="store_false", default=None,
    )
    parser.add_argument(
        "--angle-speed-profile", "--spiral-angle-speed-profile",
        dest="spiral_angle_speed_profile", type=_cli_angle_speed_profile,
    )
    parser.add_argument("--sample-step", dest="sample_step_mm", type=float)
    parser.add_argument("--keep-copy", dest="keep_processed_copy", action="store_true", default=None)
    parser.add_argument("--no-keep-copy", dest="keep_processed_copy", action="store_false", default=None)
    parser.add_argument("--secondary-mode", dest="secondary_mode", choices=["full_loop", "local_window"])
    parser.add_argument("--secondary-window", dest="secondary_window_mm", type=float)
    parser.add_argument("--secondary-start-offset", dest="secondary_start_offset_abs_mm", type=float)
    parser.add_argument("--enable-secondary", dest="secondary_pass_enabled", action="store_true", default=None)
    parser.add_argument("--disable-secondary", dest="secondary_pass_enabled", action="store_false", default=None)
    parser.add_argument("gcode_path")
    args = parser.parse_args(list(argv))
    merged = dict(config)
    for key, value in vars(args).items():
        if key == "gcode_path" or value is None:
            continue
        merged[key] = value
    return merged, args.gcode_path


def parse_line(raw: str) -> GCodeLine:
    comment = ""
    code = raw.rstrip("\n")
    if ";" in code:
        code, comment = code.split(";", 1)
        comment = ";" + comment
    code = code.strip()
    if not code:
        return GCodeLine(raw=raw, command=None, args={}, comment=comment)
    parts = code.split()
    command = parts[0].upper()
    args: Dict[str, float] = {}
    for token in parts[1:]:
        if len(token) < 2:
            continue
        axis = token[0].upper()
        try:
            args[axis] = float(token[1:])
        except ValueError:
            continue
    return GCodeLine(raw=raw, command=command, args=args, comment=comment)


@lru_cache(maxsize=8192)
def format_float(value: float) -> str:
    # 整数快速路径（进给/整 Z 值占多数，避免格式化+rstrip）
    iv = int(value)
    if value == iv:
        return str(iv)
    text = f"{value:.5f}"
    text = text.rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


_format_float_cached = format_float  # 兼容旧引用


def render_line(line: GCodeLine) -> str:
    if not line.command:
        return line.raw if line.raw.endswith("\n") else line.raw + "\n"
    return render_command(line.command, line.args, line.comment)


def render_command(command: str, args: Dict[str, float], comment: str = "") -> str:
    parts = [command]
    if "X" in args:
        parts.append(f"X{format_float(args['X'])}")
    if "Y" in args:
        parts.append(f"Y{format_float(args['Y'])}")
    if "Z" in args:
        parts.append(f"Z{format_float(args['Z'])}")
    if "E" in args:
        parts.append(f"E{format_float(args['E'])}")
    if "F" in args:
        parts.append(f"F{format_float(args['F'])}")
    for key, value in args.items():
        if key not in ("X", "Y", "Z", "E", "F"):
            parts.append(f"{key}{format_float(value)}")
    body = " ".join(parts)
    if comment:
        return body + " " + comment + "\n"
    return body + "\n"


def render_command_with_e(command: str, args: Dict[str, float], e_value: float, comment: str = "") -> str:
    parts = [command]
    if "X" in args:
        parts.append(f"X{format_float(args['X'])}")
    if "Y" in args:
        parts.append(f"Y{format_float(args['Y'])}")
    if "Z" in args:
        parts.append(f"Z{format_float(args['Z'])}")
    parts.append(f"E{format_float(e_value)}")
    if "F" in args:
        parts.append(f"F{format_float(args['F'])}")
    for key, value in args.items():
        if key not in ("X", "Y", "Z", "E", "F"):
            parts.append(f"{key}{format_float(value)}")
    body = " ".join(parts)
    if comment:
        return body + " " + comment + "\n"
    return body + "\n"


class TimeEstimator:
    """加速度感知的打印时间估计（仅统计用，不影响输出 G-code）。

    纯 distance/feed 模型在密集短段（0.2mm 细分）上严重低估——喷头在
    短段内根本加速不到指令速度。本模型与主流固件规划器一致：
    - 每段梯形加减速，加速度跟随 M204（S/P）；
    - 相邻段按夹角限制过弯速度（junction deviation，由方形拐角
      速度 SCV 换算；90° 拐角恰好限到 SCV）；
    - 分块前瞻（反向/正向扫描），摊销 O(1)/段；
    - 纯 E 移动（回抽/回填）按 |dE|/速度 计时并打断前瞻链。
    """

    _SQRT2M1 = math.sqrt(2.0) - 1.0

    def __init__(self, accel: float = 5000.0, scv: float = 5.0) -> None:
        self.accel = accel
        self.scv2 = scv * scv
        self.time = 0.0
        self._queue: List[Tuple[float, float, float, float]] = []  # (L, v_max^2, entry_cap^2, accel)
        self._ux = self._uy = self._uz = 0.0
        self._has_dir = False
        self._window = 256
        self._carry_v2 = 0.0

    def set_accel(self, accel: float) -> None:
        if accel > 0:
            self.accel = accel

    def move(self, dx: float, dy: float, dz: float, dist: float, v: float) -> None:
        """一段移动。v 为 mm/s；dist 为 3D 距离。"""
        if dist <= 1e-9 or v <= 0:
            return
        inv = 1.0 / dist
        ux, uy, uz = dx * inv, dy * inv, dz * inv
        a = self.accel
        v_max2 = v * v
        if not self._has_dir:
            entry_cap2 = 0.0
        else:
            cos_t = -(self._ux * ux + self._uy * uy + self._uz * uz)
            if cos_t > 0.999999:
                entry_cap2 = 0.0  # 折返
            elif cos_t < -0.999999:
                entry_cap2 = v_max2  # 共线
            else:
                sin_half = math.sqrt(0.5 * (1.0 - cos_t))
                r_jd = sin_half / (1.0 - sin_half)
                entry_cap2 = r_jd * self._SQRT2M1 * self.scv2
                cos_half = math.sqrt(0.5 * (1.0 + cos_t))
                if cos_half > 1e-9:
                    # 向心加速度限制（短段小角度时起主导）
                    centripetal_cap2 = 0.5 * dist * (sin_half / cos_half) * a
                    if centripetal_cap2 < entry_cap2:
                        entry_cap2 = centripetal_cap2
                if v_max2 < entry_cap2:
                    entry_cap2 = v_max2
        self._ux, self._uy, self._uz = ux, uy, uz
        self._has_dir = True
        self._queue.append((dist, v_max2, entry_cap2, a))
        if len(self._queue) >= 2 * self._window:
            self._plan(self._window)

    def e_only(self, de: float, v: float) -> None:
        """纯 E 移动：喷头静止，打断前瞻链。"""
        self.flush()
        if de > 0 and v > 0:
            self.time += de / v

    def _plan(self, finalize: int) -> None:
        q = self._queue
        n = len(q)
        if n == 0:
            return
        finalize = min(finalize, n)
        # 反向扫描：末段出口按 0 保守处理（finalize < n 时误差只影响
        # 远端，被下一块修正）
        entry2 = [0.0] * (n + 1)
        for i in range(n - 1, -1, -1):
            dist, v_max2, entry_cap2, a = q[i]
            value = entry_cap2
            if v_max2 < value:
                value = v_max2
            reachable = entry2[i + 1] + 2.0 * a * dist
            if reachable < value:
                value = reachable
            entry2[i] = value
        # 正向扫描 + 计时
        ve2 = self._carry_v2
        for i in range(finalize):
            dist, v_max2, entry_cap2, a = q[i]
            entry_limit = entry2[i]
            if entry_limit < ve2:
                ve2 = entry_limit
            vx2 = entry2[i + 1]
            reachable = ve2 + 2.0 * a * dist
            if reachable < vx2:
                vx2 = reachable
            vp2 = v_max2
            reachable = 0.5 * (ve2 + vx2) + a * dist
            if reachable < vp2:
                vp2 = reachable
            ve = math.sqrt(ve2)
            vx = math.sqrt(vx2)
            vp = math.sqrt(vp2)
            t = (vp - ve) / a + (vp - vx) / a
            d_cruise = dist - (vp2 - ve2) / (2.0 * a) - (vp2 - vx2) / (2.0 * a)
            if d_cruise > 1e-9 and vp > 1e-9:
                t += d_cruise / vp
            self.time += t
            ve2 = vx2
        self._carry_v2 = ve2
        del q[:finalize]

    def flush(self) -> None:
        self._plan(len(self._queue))
        self._carry_v2 = 0.0
        self._has_dir = False


def annotate(lines: Sequence[GCodeLine]) -> Tuple[List[LineInfo], float, float, float]:
    infos: List[LineInfo] = []
    x = y = z = 0.0
    e = 0.0
    feed = DEFAULT_FEED
    relative = False
    coord_relative = False
    est = TimeEstimator()
    base_path = 0.0
    base_ext = 0.0
    for idx, line in enumerate(lines):
        cmd = line.command
        kind = "other"
        de = 0.0
        x0, y0, z0 = x, y, z
        if cmd == "M204":
            a = line.args.get("P", line.args.get("S", 0.0))
            est.set_accel(a)
        elif cmd == "M82":
            relative = False
            kind = "m82"
        elif cmd == "M83":
            relative = True
            kind = "m83"
        elif cmd == "G90":
            coord_relative = False
            kind = "g90"
        elif cmd == "G91":
            coord_relative = True
            kind = "g91"
        elif cmd == "G92":
            if "X" in line.args:
                x = line.args["X"]
            if "Y" in line.args:
                y = line.args["Y"]
            if "Z" in line.args:
                z = line.args["Z"]
            if "E" in line.args:
                e = line.args["E"]
            kind = "g92"
        elif cmd in ("G0", "G1", "G2", "G3"):
            kind = "move"
            if "F" in line.args and line.args["F"] > 0:
                feed = line.args["F"]
            if coord_relative:
                nx = x + line.args.get("X", 0.0)
                ny = y + line.args.get("Y", 0.0)
                nz = z + line.args.get("Z", 0.0)
            else:
                nx = line.args.get("X", x)
                ny = line.args.get("Y", y)
                nz = line.args.get("Z", z)
            if "E" in line.args:
                if relative:
                    de = line.args["E"]
                else:
                    de = line.args["E"] - e
                    e = line.args["E"]
            dx, dy, dz = nx - x, ny - y, nz - z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance > 0 and feed > 0:
                est.move(dx, dy, dz, distance, feed / 60.0)
            elif de != 0.0 and feed > 0:
                est.e_only(abs(de), feed / 60.0)
            base_path += distance
            if de > 0:
                base_ext += de
            x, y, z = nx, ny, nz
        infos.append(
            LineInfo(
                index=idx,
                line=line,
                kind=kind,
                x0=x0,
                y0=y0,
                z0=z0,
                x1=x,
                y1=y,
                z1=z,
                de=de,
                feed=feed,
            )
        )
    est.flush()
    return infos, est.time, base_path, base_ext


def subdivide_loop(loop: Loop, step: float) -> Loop:
    """把回路长段按 step 细分（同一原始行拆成多个子段，de 按长度均分）。

    细分后的回路用于生成螺旋/斜接缝/二次整形路径：Z 爬升与 XY 插值在
    长直边中间也连续，而不是只在原始顶点处取值。"""
    if step <= 0.05:
        step = 0.05
    segments: List[Segment] = []
    for seg in loop.segments:
        n = max(1, math.ceil(seg.length / step - 1e-9))
        if n == 1:
            segments.append(seg)
            continue
        for i in range(n):
            t0, t1 = i / n, (i + 1) / n
            a = (
                seg.x0 + (seg.x1 - seg.x0) * t0,
                seg.y0 + (seg.y1 - seg.y0) * t0,
                seg.z0 + (seg.z1 - seg.z0) * t0,
            )
            b = (
                seg.x0 + (seg.x1 - seg.x0) * t1,
                seg.y0 + (seg.y1 - seg.y0) * t1,
                seg.z0 + (seg.z1 - seg.z0) * t1,
            )
            segments.append(Segment(
                x0=a[0], y0=a[1], z0=a[2],
                x1=b[0], y1=b[1], z1=b[2],
                de=seg.de / n, length=seg.length / n, line_index=seg.line_index,
            ))
    sub = Loop(layer_index=loop.layer_index, z=loop.z, segments=segments,
               end_insert_index=loop.end_insert_index, has_arc=loop.has_arc)
    sub.build_cum()
    return sub


def generated_sample_step(loop: Loop, config: Dict[str, Any], sample_step: float) -> float:
    """Sampling step for generated spiral/secondary geometry.

    `sample_step_mm` also controls arc expansion while reading the slicer's path.  Applying very
    fine values directly to long generated loops can create millions of output rows with little
    practical gain, because the slicer has already approximated the contour.  Generated paths
    stay at least one line width apart and use a coarser target on very long loops while keeping
    all original slicer vertices.
    """
    width_step = max(0.05, fnum(config, "path_width_mm"))
    budget_step = loop.total_length / MAX_GENERATED_LOOP_SEGMENTS if loop.total_length > 0 else 0.0
    return max(sample_step, width_step, budget_step)


def expand_arc(info: LineInfo, step: float) -> Optional[List[Segment]]:
    """把 G2/G3 圆弧展开为直线子段（IJ 圆心格式；无 IJ 返回 None）。"""
    args = info.line.args
    if "I" not in args and "J" not in args:
        return None
    x0, y0, z0 = info.x0, info.y0, info.z0
    x1, y1, z1 = info.x1, info.y1, info.z1
    cx, cy = x0 + args.get("I", 0.0), y0 + args.get("J", 0.0)
    r0 = math.hypot(x0 - cx, y0 - cy)
    r1 = math.hypot(x1 - cx, y1 - cy)
    if r0 <= 1e-6 or abs(r0 - r1) > max(0.05, 0.01 * r0):
        return None
    a0 = math.atan2(y0 - cy, x0 - cx)
    a1 = math.atan2(y1 - cy, x1 - cx)
    cw = info.line.command == "G2"
    sweep = a1 - a0
    if cw:
        while sweep >= -1e-9:
            sweep -= 2 * math.pi
    else:
        while sweep <= 1e-9:
            sweep += 2 * math.pi
    arc_len = abs(sweep) * r0
    n = max(2, math.ceil(arc_len / max(step, 0.05)))
    segments: List[Segment] = []
    prev = (x0, y0, z0)
    for i in range(1, n + 1):
        t = i / n
        ang = a0 + sweep * t
        pt = (cx + r0 * math.cos(ang), cy + r0 * math.sin(ang), z0 + (z1 - z0) * t)
        length = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
        segments.append(Segment(
            x0=prev[0], y0=prev[1], z0=prev[2],
            x1=pt[0], y1=pt[1], z1=pt[2],
            de=info.de / n, length=length, line_index=info.index,
        ))
        prev = pt
    return segments


def build_loops(
    infos: Sequence[LineInfo],
    sample_step: float = 1.0,
    include_overhang_starts: bool = False,
) -> Tuple[List[Loop], int]:
    loops: List[Loop] = []
    arc_loops_skipped = 0
    current_segments: List[Segment] = []
    layer_index = -1
    in_outer = False
    arc_tainted = False
    travel_acc = 0.0

    def flush(loop_end_index: int) -> None:
        nonlocal current_segments, arc_tainted, arc_loops_skipped, travel_acc
        travel_acc = 0.0
        if current_segments:
            if arc_tainted:
                arc_loops_skipped += 1
            elif len(current_segments) >= 2:
                loop = Loop(
                    layer_index=max(layer_index, 0),
                    z=current_segments[0].z0,
                    segments=current_segments,
                    end_insert_index=loop_end_index,
                )
                loop.build_cum()
                loops.append(loop)
        current_segments = []
        arc_tainted = False

    for info in infos:
        line = info.line
        marker = line.comment[1:].strip().upper() if line.comment.startswith(";") else ""
        if marker:
            if any(token in marker for token in LAYER_MARKERS):
                if in_outer:
                    flush(info.index)
                    in_outer = False
                layer_index += 1
            if any(m in marker for m in OUTER_WALL_MARKERS):
                in_outer = True
                continue
            if any(m in marker for m in OVERHANG_WALL_MARKERS):
                if include_overhang_starts and not in_outer:
                    in_outer = True
                # 悬垂墙标记：仅当外墙回路已开始时视为同一回路的延续；
                # 出现在内墙等其他特征之后时不属于外墙，不处理
                continue
            if in_outer and "TYPE:" in marker:
                flush(info.index)
                in_outer = False
        if not in_outer or info.kind != "move":
            if in_outer and line.command == "G92" and any(axis in line.args for axis in ("X", "Y", "Z")):
                flush(info.index)
            continue
        if line.command in ("G2", "G3"):
            expanded = expand_arc(info, sample_step)
            if info.de > 1e-9:
                if expanded is None:
                    arc_tainted = True
                else:
                    current_segments.extend(expanded)
            else:
                length = math.hypot(info.x1 - info.x0, info.y1 - info.y0)
                if expanded is not None:
                    length = sum(seg.length for seg in expanded)
                travel_acc += length
                if current_segments and travel_acc > TRAVEL_GAP_MM:
                    flush(info.index)
            continue
        length = math.hypot(info.x1 - info.x0, info.y1 - info.y0)
        if info.de <= 1e-9:
            # 回抽/空移：累计连续非挤出移动的总距离（擦拭、z-hop 常被拆成
            # 一串小步，单步不超阈值但整体跨到另一个岛），超过间隙即截断
            travel_acc += length
            if current_segments and travel_acc > TRAVEL_GAP_MM:
                flush(info.index)
            continue
        travel_acc = 0.0
        if length <= 0.001:
            continue
        current_segments.append(
            Segment(
                x0=info.x0, y0=info.y0, z0=info.z0,
                x1=info.x1, y1=info.y1, z1=info.z1,
                de=info.de,
                length=length,
                line_index=info.index,
            )
        )
    flush(len(infos))
    return loops, arc_loops_skipped


def build_inner_wall_segments(
    infos: Sequence[LineInfo],
    sample_step: float = 1.0,
) -> Dict[int, List[Segment]]:
    by_layer: Dict[int, List[Segment]] = {}
    layer_index = -1
    in_inner = False
    for info in infos:
        line = info.line
        marker = line.comment[1:].strip().upper() if line.comment.startswith(";") else ""
        if marker:
            if any(token in marker for token in LAYER_MARKERS):
                in_inner = False
                layer_index += 1
            if any(m in marker for m in INNER_WALL_MARKERS):
                in_inner = True
                continue
            if "TYPE:" in marker or "FEATURE:" in marker:
                in_inner = False
        if not in_inner or info.kind != "move" or info.de <= 1e-9:
            continue
        if line.command in ("G2", "G3"):
            expanded = expand_arc(info, sample_step)
            if expanded is not None:
                by_layer.setdefault(max(layer_index, 0), []).extend(expanded)
            continue
        length = math.hypot(info.x1 - info.x0, info.y1 - info.y0)
        if length <= 0.001:
            continue
        by_layer.setdefault(max(layer_index, 0), []).append(
            Segment(
                x0=info.x0, y0=info.y0, z0=info.z0,
                x1=info.x1, y1=info.y1, z1=info.z1,
                de=info.de, length=length, line_index=info.index,
            )
        )
    return by_layer


def segment_scale(position: float, total_length: float, config: Dict[str, Any]) -> float:
    """退化模式（无 Z 渐变）：接缝两端流量在 min 与 base 之间线性渐变。"""
    scarf = max(0.0, fnum(config, "scarf_length_mm"))
    base = fnum(config, "primary_flow_scale")
    seam_min = fnum(config, "primary_seam_min_scale")
    if scarf <= 0.0 or total_length <= 0.0 or total_length <= 2 * scarf:
        return base
    if position <= scarf:
        factor = max(0.0, min(1.0, position / scarf))
        return seam_min + (base - seam_min) * factor
    if position >= total_length - scarf:
        factor = max(0.0, min(1.0, (total_length - position) / scarf))
        return seam_min + (base - seam_min) * factor
    return base


def point_along_loop(loop: Loop, distance_mm: float) -> Tuple[Tuple[float, float, float], int]:
    loop = closed_loop_geometry(loop)
    total = loop.total_length
    if total <= 0:
        seg0 = loop.segments[0]
        return (seg0.x0, seg0.y0, seg0.z0), 0
    remaining = distance_mm % total
    index = min(bisect_right(loop.cum, remaining), len(loop.segments) - 1)
    seg_start_dist = loop.cum[index - 1] if index > 0 else 0.0
    seg = loop.segments[index]
    ratio = (remaining - seg_start_dist) / seg.length if seg.length > 0 else 0.0
    ratio = max(0.0, min(1.0, ratio))
    point = (
        seg.x0 + (seg.x1 - seg.x0) * ratio,
        seg.y0 + (seg.y1 - seg.y0) * ratio,
        seg.z0 + (seg.z1 - seg.z0) * ratio,
    )
    return point, index


@dataclass(slots=True)
class ZonePiece:
    """接缝斜拼区的一小段：终点坐标、终点/中点弧长位置、对应的原始挤出量。"""
    x: float
    y: float
    s_end: float
    s_mid: float
    de: float
    line_index: int


def scarf_zone_pieces(loop: Loop, scarf: float, piece_len: float) -> List[ZonePiece]:
    """把回路起始 scarf 长度按 piece_len 细分；同一份细分同时用于
    起始渐变段和结尾重叠段，保证两者流量互补、总挤出精确守恒。"""
    pieces: List[ZonePiece] = []
    s0 = 0.0
    for seg in loop.segments:
        if s0 >= scarf - 1e-9:
            break
        zone_len = min(seg.length, scarf - s0)
        count = max(1, math.ceil(zone_len / piece_len - 1e-9))
        for i in range(1, count + 1):
            a = zone_len * (i - 1) / count
            b = zone_len * i / count
            ratio = b / seg.length
            pieces.append(ZonePiece(
                x=seg.x0 + (seg.x1 - seg.x0) * ratio,
                y=seg.y0 + (seg.y1 - seg.y0) * ratio,
                s_end=s0 + b,
                s_mid=s0 + (a + b) / 2,
                de=seg.de * (b - a) / seg.length,
                line_index=seg.line_index,
            ))
        s0 += seg.length
    return pieces


def build_scarf(
    loop: Loop,
    config: Dict[str, Any],
    layer_height: float,
    wall_feed: float,
) -> Tuple[Dict[int, List[Ins]], List[Ins]]:
    """立体斜接缝：返回 (起始段替换指令, 结尾重叠段指令)。

    起始 scarf 区：Z 从 z-h 渐升到 z，流量从 min 渐增到 base；
    结尾重叠段：沿同一路径在 Z=z 补互补流量（base - 起始流量），
    两个楔形上下拼合，每个位置总流量恰为 base。"""
    scarf = max(0.0, fnum(config, "scarf_length_mm"))
    base = fnum(config, "primary_flow_scale")
    seam_min = fnum(config, "primary_seam_min_scale")
    superres = max(1, int(fnum(config, "seam_superres_x")))
    piece_len = scarf / superres if superres > 0 else scarf
    z_ramp = layer_height > 0.02

    pieces = scarf_zone_pieces(loop, scarf, piece_len)
    if not pieces:
        return {}, []

    def flow_up(s: float) -> float:
        t = min(1.0, s / scarf)
        if z_ramp:
            # 间隙匹配：喷嘴到前层顶面的间隙 = t*层高，流量必须同比例，
            # 否则薄端会把多余料犁成突起（起点流量参数仅用于平面模式）
            return base * t
        return seam_min + (base - seam_min) * t

    replacements: Dict[int, List[Ins]] = {}
    for p in pieces:
        z = loop.z - layer_height * (1.0 - min(1.0, p.s_end / scarf)) if z_ramp else None
        ins = Ins(kind="extrude", x=p.x, y=p.y, z=z, de=p.de * flow_up(p.s_mid))
        replacements.setdefault(p.line_index, []).append(ins)
    # 斜拼区所在原始行的剩余部分：按主体流量补齐（Z 回到层高）。
    # 细分后同一原始行可能横跨斜拼边界，越界的所有子段都要补齐
    zone_line_indices = set(replacements.keys())
    s0 = 0.0
    for seg in loop.segments:
        s_start = s0
        s_end = s0 + seg.length
        if seg.line_index in zone_line_indices and s_end > scarf + 1e-9:
            if s_start < scarf:
                rest_de = seg.de * (s_end - scarf) / seg.length
            else:
                rest_de = seg.de
            replacements[seg.line_index].append(
                Ins(kind="extrude", x=seg.x1, y=seg.y1,
                    z=loop.z if z_ramp else None, de=rest_de * base)
            )
        s0 = s_end

    wall_feed = DEFAULT_FEED if wall_feed <= 0 else wall_feed
    # 重叠段细分与超分倍数脱钩：它在恒定 Z 上，不受 Z 步进能力限制，
    # 切细让尾端流量平滑归零，避免收尾余压棱
    overlap_pieces = scarf_zone_pieces(loop, scarf, min(piece_len, 0.4))
    overlap: List[Ins] = [Ins(kind="comment", text="; BOWP scarf overlap start\n")]
    first_seg = loop.segments[0]
    last_seg = loop.segments[-1]
    start_pt = (first_seg.x0, first_seg.y0, first_seg.z0)
    end_pt = (last_seg.x1, last_seg.y1, last_seg.z1)
    if math.hypot(end_pt[0] - start_pt[0], end_pt[1] - start_pt[1]) > 0.05:
        # 开环（被回抽/空移截断）才需要空移；闭环直接连续接打，不停顿
        overlap.append(Ins(kind="travel", x=start_pt[0], y=start_pt[1], z=loop.z))
    for p in overlap_pieces:
        overlap.append(Ins(kind="extrude", x=p.x, y=p.y, z=loop.z,
                           de=p.de * (base - flow_up(p.s_mid)), f=wall_feed))
    overlap.append(Ins(kind="comment", text="; BOWP scarf overlap end\n"))
    return replacements, overlap


def _loop_has_seam_leadin(loop: Loop) -> bool:
    if loop.seam_leadin is not None:
        return loop.seam_leadin
    if len(loop.segments) < 3:
        loop.seam_leadin = False
        return False
    first_seg = loop.segments[0]
    last_seg = loop.segments[-1]
    seam_gap = math.hypot(last_seg.x1 - first_seg.x1, last_seg.y1 - first_seg.y1)
    direct_gap = math.hypot(last_seg.x1 - first_seg.x0, last_seg.y1 - first_seg.y0)
    total = loop.total_length if loop.total_length > 0 else sum(seg.length for seg in loop.segments)
    max_leadin = max(0.3, min(LOOP_LEADIN_MAX_MM, total * 0.03))
    loop.seam_leadin = (
        seam_gap <= LOOP_LEADIN_CLOSE_TOL_MM
        and first_seg.length <= max_leadin
        and seam_gap + 0.02 < direct_gap
    )
    return loop.seam_leadin


def loop_is_closed(loop: Loop) -> bool:
    if not loop.segments:
        return False
    if _loop_has_seam_leadin(loop):
        return True
    first_seg = loop.segments[0]
    last_seg = loop.segments[-1]
    return math.hypot(last_seg.x1 - first_seg.x0, last_seg.y1 - first_seg.y0) <= LOOP_CLOSE_TOL_MM


def loop_z_span(loop: Loop) -> float:
    if not loop.segments:
        return 0.0
    z_min = min(min(seg.z0, seg.z1) for seg in loop.segments)
    z_max = max(max(seg.z0, seg.z1) for seg in loop.segments)
    return z_max - z_min


def closed_loop_geometry(loop: Loop) -> Loop:
    """Return the printable closed contour, excluding a short Orca seam lead-in when present."""
    if loop.geometry_resolved:
        return loop.geometry_loop if loop.geometry_loop is not None else loop
    if not _loop_has_seam_leadin(loop):
        loop.geometry_resolved = True
        return loop
    if loop.geometry_loop is not None:
        loop.geometry_resolved = True
        return loop.geometry_loop
    kept = loop.segments[1:]
    if len(kept) < 2:
        loop.geometry_resolved = True
        return loop
    original_de = sum(seg.de for seg in loop.segments)
    kept_de = sum(seg.de for seg in kept)
    de_scale = original_de / kept_de if kept_de > 1e-12 else 1.0
    segments = [
        Segment(
            x0=seg.x0, y0=seg.y0, z0=seg.z0,
            x1=seg.x1, y1=seg.y1, z1=seg.z1,
            de=seg.de * de_scale,
            length=seg.length,
            line_index=seg.line_index,
        )
        for seg in kept
    ]
    out = Loop(
        layer_index=loop.layer_index,
        z=loop.z,
        segments=segments,
        end_insert_index=loop.end_insert_index,
        has_arc=loop.has_arc,
    )
    out.build_cum()
    loop.geometry_loop = out
    loop.geometry_resolved = True
    return out


def loop_signed_area(loop: Loop) -> float:
    geo = closed_loop_geometry(loop)
    return 0.5 * sum(seg.x0 * seg.y1 - seg.x1 * seg.y0 for seg in geo.segments)


def loop_vertices_xy(loop: Loop) -> List[Tuple[float, float]]:
    geo = closed_loop_geometry(loop)
    if geo.verts_xy is None:
        geo.verts_xy = [(seg.x1, seg.y1) for seg in geo.segments]
    return geo.verts_xy


def loop_vertex_normals(loop: Loop) -> List[Tuple[float, float]]:
    geo = closed_loop_geometry(loop)
    if geo.normals_xy is not None:
        return geo.normals_xy
    verts = loop_vertices_xy(geo)
    n = len(verts)
    normals: List[Tuple[float, float]] = []
    for i in range(n):
        a = verts[(i - 1) % n]
        b = verts[(i + 1) % n]
        tx, ty = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(tx, ty)
        normals.append((0.0, 0.0) if norm <= 1e-9 else (ty / norm, -tx / norm))
    geo.normals_xy = normals
    return normals


def loop_centroid(loop: Loop) -> Tuple[float, float]:
    if loop.centroid_xy is not None:
        return loop.centroid_xy
    verts = loop_vertices_xy(loop)
    sx = sum(x for x, _ in verts)
    sy = sum(y for _, y in verts)
    n = len(verts)
    loop.centroid_xy = (sx / n, sy / n)
    return loop.centroid_xy


GRID_CELL_MM = 2.0
GRID_CELL_INV = 1.0 / GRID_CELL_MM


def _loop_grid(loop: Loop) -> Dict[Tuple[int, int], List[int]]:
    """按段包围盒建立哈希网格（懒构建，缓存在 loop.grid）。"""
    loop = closed_loop_geometry(loop)
    if loop.grid is not None:
        return loop.grid
    grid: Dict[Tuple[int, int], List[int]] = {}
    inv = 1.0 / GRID_CELL_MM
    for idx, seg in enumerate(loop.segments):
        x0 = min(seg.x0, seg.x1)
        x1 = max(seg.x0, seg.x1)
        y0 = min(seg.y0, seg.y1)
        y1 = max(seg.y0, seg.y1)
        for gx in range(int(x0 * inv), int(x1 * inv) + 1):
            for gy in range(int(y0 * inv), int(y1 * inv) + 1):
                grid.setdefault((gx, gy), []).append(idx)
    loop.grid = grid
    return grid


def _nearest_in_segments(px: float, py: float, segments: Sequence[Segment],
                         indices: Optional[Sequence[int]] = None) -> Tuple[float, float, float]:
    best_x, best_y = segments[0].x0, segments[0].y0
    best_d = float("inf")
    it = indices if indices is not None else range(len(segments))
    for j in it:
        seg = segments[j]
        ax, ay = seg.x0, seg.y0
        bx, by = seg.x1, seg.y1
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-12:
            t = 0.0
        else:
            t = ((px - ax) * abx + (py - ay) * aby) / ab2
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0
        qx, qy = ax + abx * t, ay + aby * t
        dx, dy = qx - px, qy - py
        d = dx * dx + dy * dy
        if d < best_d:
            best_d = d
            best_x, best_y = qx, qy
    return best_x, best_y, best_d


def _loop_nearest_data(loop: Loop) -> List[Tuple[float, float, float, float, float]]:
    loop = closed_loop_geometry(loop)
    if loop.nearest_data is None:
        data: List[Tuple[float, float, float, float, float]] = []
        for seg in loop.segments:
            abx = seg.x1 - seg.x0
            aby = seg.y1 - seg.y0
            data.append((seg.x0, seg.y0, abx, aby, abx * abx + aby * aby))
        loop.nearest_data = data
    return loop.nearest_data


def _nearest_in_loop_data(
    px: float,
    py: float,
    data: Sequence[Tuple[float, float, float, float, float]],
    indices: Optional[Sequence[int]] = None,
) -> Tuple[float, float, int, float]:
    best_x, best_y = data[0][0], data[0][1]
    best_i = 0
    best_d = float("inf")
    it = indices if indices is not None else range(len(data))
    for j in it:
        ax, ay, abx, aby, ab2 = data[j]
        if ab2 <= 1e-12:
            t = 0.0
        else:
            t = ((px - ax) * abx + (py - ay) * aby) / ab2
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0
        qx, qy = ax + abx * t, ay + aby * t
        dx, dy = qx - px, qy - py
        d = dx * dx + dy * dy
        if d < best_d:
            best_d = d
            best_x, best_y = qx, qy
            best_i = j
    return best_x, best_y, best_i, best_d


class SegmentGrid:
    def __init__(self, segments: Sequence[Segment], cell_mm: float = GRID_CELL_MM) -> None:
        self.segments = list(segments)
        self.cell_mm = max(0.5, cell_mm)
        self.grid: Dict[Tuple[int, int], List[int]] = {}
        inv = 1.0 / self.cell_mm
        for idx, seg in enumerate(self.segments):
            x0 = min(seg.x0, seg.x1)
            x1 = max(seg.x0, seg.x1)
            y0 = min(seg.y0, seg.y1)
            y1 = max(seg.y0, seg.y1)
            for gx in range(math.floor(x0 * inv), math.floor(x1 * inv) + 1):
                for gy in range(math.floor(y0 * inv), math.floor(y1 * inv) + 1):
                    self.grid.setdefault((gx, gy), []).append(idx)

    def nearest_within(self, px: float, py: float, radius: float) -> Optional[Tuple[float, float, float]]:
        if not self.segments or radius <= 0.0:
            return None
        if len(self.segments) <= 24:
            qx, qy, d2 = _nearest_in_segments(px, py, self.segments)
            return (qx, qy, d2) if d2 <= radius * radius else None
        inv = 1.0 / self.cell_mm
        gx0 = math.floor((px - radius) * inv)
        gx1 = math.floor((px + radius) * inv)
        gy0 = math.floor((py - radius) * inv)
        gy1 = math.floor((py + radius) * inv)
        idxs: List[int] = []
        visited = bytearray(len(self.segments))
        for gx in range(gx0, gx1 + 1):
            for gy in range(gy0, gy1 + 1):
                for idx in self.grid.get((gx, gy), ()):
                    if not visited[idx]:
                        visited[idx] = 1
                        idxs.append(idx)
        if not idxs:
            return None
        qx, qy, d2 = _nearest_in_segments(px, py, self.segments, idxs)
        return (qx, qy, d2) if d2 <= radius * radius else None


def loop_lacks_inner_wall_support(
    loop: Loop,
    inner_index: Optional[SegmentGrid],
    path_width: float,
    sample_step: float,
) -> bool:
    """Return True when an outer-wall loop has a meaningful single-wall span."""
    if not loop_is_closed(loop):
        return False
    loop = closed_loop_geometry(loop)
    total = loop.total_length
    if total <= 1e-9:
        return False
    if inner_index is None or not inner_index.segments:
        return True

    support_radius = max(0.8, 3.0 * max(path_width, 0.05))
    spacing = max(0.25, min(1.0, max(sample_step * 2.0, path_width)))
    n = max(8, math.ceil(total / spacing))
    step_len = total / n
    flags: List[bool] = []
    for i in range(n):
        (px, py, _), _ = point_along_loop(loop, (i + 0.5) * step_len)
        hit = inner_index.nearest_within(px, py, support_radius)
        if hit is None:
            flags.append(True)
            continue
        qx, qy, _ = hit
        # A nearby inner wall from another island must not support this loop.
        flags.append(not point_in_loop(qx, qy, loop))

    unsupported_len = sum(step_len for flag in flags if flag)
    if unsupported_len <= 1e-9:
        return False
    if all(flags):
        max_run = total
    else:
        max_run = 0.0
        run = 0.0
        for flag in flags + flags:
            if flag:
                run = min(total, run + step_len)
                if run > max_run:
                    max_run = run
            else:
                run = 0.0

    min_run = max(1.0, 2.5 * max(path_width, 0.05))
    return max_run >= min_run and unsupported_len / total >= 0.02


def nearest_on_loop(point: Tuple[float, float], loop: Loop) -> Tuple[float, float]:
    """Return the nearest continuous point on a loop."""
    loop = closed_loop_geometry(loop)
    px, py = point
    segments = loop.segments
    nearest_data = loop.nearest_data
    if nearest_data is None:
        nearest_data = _loop_nearest_data(loop)
    if len(segments) <= 24:
        qx, qy, _, _ = _nearest_in_loop_data(px, py, nearest_data)
        return qx, qy
    grid = loop.grid
    if grid is None:
        grid = _loop_grid(loop)
    inv = GRID_CELL_INV
    cgx, cgy = int(px * inv), int(py * inv)
    best_x = best_y = 0.0
    best_d = float("inf")
    visited = bytearray(len(segments))
    for ring in range(512):
        if ring >= 2:
            stop_distance = (ring - 1) * GRID_CELL_MM
            if best_d < stop_distance * stop_distance:
                break
        if ring == 0:
            cells = [(cgx, cgy)]
        else:
            cells = []
            for k in range(-ring, ring + 1):
                cells.append((cgx + k, cgy - ring))
                cells.append((cgx + k, cgy + ring))
            for k in range(-ring + 1, ring):
                cells.append((cgx - ring, cgy + k))
                cells.append((cgx + ring, cgy + k))
        idxs: List[int] = []
        for cell in cells:
            values = grid.get(cell)
            if values:
                for index in values:
                    if not visited[index]:
                        visited[index] = 1
                        idxs.append(index)
        if idxs:
            qx, qy, _, distance = _nearest_in_loop_data(px, py, nearest_data, idxs)
            if distance < best_d:
                best_d = distance
                best_x, best_y = qx, qy
    if best_d == float("inf"):
        qx, qy, _, _ = _nearest_in_loop_data(px, py, nearest_data)
        return qx, qy
    return best_x, best_y


def nearest_on_loop_with_index(point: Tuple[float, float], loop: Loop) -> Tuple[float, float, int, float]:
    """返回回路折线上离 point 最近的点、段索引与平方距离。"""
    loop = closed_loop_geometry(loop)
    px, py = point
    segments = loop.segments
    nearest_data = loop.nearest_data
    if nearest_data is None:
        nearest_data = _loop_nearest_data(loop)
    if len(segments) <= 24:
        return _nearest_in_loop_data(px, py, nearest_data)
    grid = loop.grid
    if grid is None:
        grid = _loop_grid(loop)
    inv = GRID_CELL_INV
    cgx, cgy = int(px * inv), int(py * inv)
    best_x = best_y = 0.0
    best_i = 0
    best_d2 = float("inf")
    visited = bytearray(len(segments))
    for ring in range(512):
        if ring >= 2:
            stop_distance = (ring - 1) * GRID_CELL_MM
            if best_d2 < stop_distance * stop_distance:
                break
        if ring == 0:
            cells = [(cgx, cgy)]
        else:
            cells = []
            for k in range(-ring, ring + 1):
                cells.append((cgx + k, cgy - ring))
                cells.append((cgx + k, cgy + ring))
            for k in range(-ring + 1, ring):
                cells.append((cgx - ring, cgy + k))
                cells.append((cgx + ring, cgy + k))
        for cell in cells:
            for i in grid.get(cell, ()):
                if visited[i]:
                    continue
                visited[i] = 1
                ax, ay, abx, aby, ab2 = nearest_data[i]
                if ab2 <= 1e-12:
                    t = 0.0
                else:
                    t = ((px - ax) * abx + (py - ay) * aby) / ab2
                    if t < 0.0:
                        t = 0.0
                    elif t > 1.0:
                        t = 1.0
                qx, qy = ax + abx * t, ay + aby * t
                dx, dy = qx - px, qy - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_x, best_y, best_i, best_d2 = qx, qy, i, d2
    if best_d2 == float("inf"):
        best_x, best_y, best_i, best_d2 = _nearest_in_loop_data(px, py, nearest_data)
    return best_x, best_y, best_i, best_d2


def point_in_loop(px: float, py: float, loop: Loop) -> bool:
    """射线法判断点是否在回路多边形内（对非凸轮廓正确）。"""
    loop = closed_loop_geometry(loop)
    inside = False
    for seg in loop.segments:
        ax, ay = seg.x0, seg.y0
        bx, by = seg.x1, seg.y1
        if (ay > py) != (by > py):
            t = (py - ay) / seg.dy
            if ax + seg.dx * t > px:
                inside = not inside
    return inside


def merge_intervals(intervals: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if not intervals:
        return []
    ordered = [
        (a, b) if a <= b else (b, a)
        for a, b in intervals
        if abs(b - a) > 1e-9
    ]
    if not ordered:
        return []
    ordered.sort()
    merged = [ordered[0]]
    for a, b in ordered[1:]:
        la, lb = merged[-1]
        if a <= lb + 1e-9:
            if b > lb:
                merged[-1] = (la, b)
        else:
            merged.append((a, b))
    return merged


def expand_intervals(intervals: Sequence[Tuple[float, float]], amount: float) -> List[Tuple[float, float]]:
    if amount <= 0:
        return list(intervals)
    return merge_intervals((a - amount, b + amount) for a, b in intervals)


def subtract_intervals(
    intervals: Sequence[Tuple[float, float]],
    cuts: Sequence[Tuple[float, float]],
    cuts_are_forward: bool = False,
    cuts_are_sorted: bool = False,
) -> List[Tuple[float, float]]:
    if not intervals:
        return []
    if not cuts:
        return list(intervals)
    if cuts_are_forward:
        ordered_cuts = cuts if cuts_are_sorted else sorted(cuts)
        merged_cuts = [ordered_cuts[0]]
        for ca, cb in ordered_cuts[1:]:
            la, lb = merged_cuts[-1]
            if ca <= lb + 1e-9:
                if cb > lb:
                    merged_cuts[-1] = (la, cb)
            else:
                merged_cuts.append((ca, cb))
    else:
        merged_cuts = merge_intervals(cuts)
    if len(intervals) == 1:
        start, end = intervals[0]
        cursor = start
        out: List[Tuple[float, float]] = []
        for ca, cb in merged_cuts:
            if cb <= cursor:
                continue
            if ca >= end:
                break
            if ca > cursor:
                out.append((cursor, ca))
            if cb >= end:
                cursor = end
                break
            cursor = cb
        if cursor < end:
            out.append((cursor, end))
        return out
    out = list(intervals)
    for ca, cb in merged_cuts:
        next_out: List[Tuple[float, float]] = []
        for a, b in out:
            if cb <= a or ca >= b:
                next_out.append((a, b))
                continue
            if ca > a:
                next_out.append((a, ca))
            if cb < b:
                next_out.append((cb, b))
        out = next_out
        if not out:
            break
    return out


def intersect_intervals(
    left: Sequence[Tuple[float, float]],
    right: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    a = merge_intervals(left)
    b = merge_intervals(right)
    out: List[Tuple[float, float]] = []
    i = j = 0
    while i < len(a) and j < len(b):
        lo = max(a[i][0], b[j][0])
        hi = min(a[i][1], b[j][1])
        if hi - lo > 1e-9:
            out.append((lo, hi))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def intersect_sorted_intervals(
    left: Sequence[Tuple[float, float]],
    right: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    """Intersect already normalized, sorted, non-overlapping interval lists."""
    if not left or not right:
        return []
    left_count = len(left)
    right_count = len(right)
    if left_count == 1 and right_count == 1:
        la, lb = left[0]
        ra, rb = right[0]
        lo = la if la >= ra else ra
        hi = lb if lb <= rb else rb
        return [(lo, hi)] if hi - lo > 1e-9 else []
    if left_count == 1:
        la, lb = left[0]
        out: List[Tuple[float, float]] = []
        for ra, rb in right:
            lo = la if la >= ra else ra
            hi = lb if lb <= rb else rb
            if hi - lo > 1e-9:
                out.append((lo, hi))
        return out
    if right_count == 1:
        ra, rb = right[0]
        out = []
        for la, lb in left:
            lo = la if la >= ra else ra
            hi = lb if lb <= rb else rb
            if hi - lo > 1e-9:
                out.append((lo, hi))
        return out
    out: List[Tuple[float, float]] = []
    i = j = 0
    while i < left_count and j < right_count:
        la, lb = left[i]
        ra, rb = right[j]
        lo = la if la >= ra else ra
        hi = lb if lb <= rb else rb
        if hi - lo > 1e-9:
            out.append((lo, hi))
        if lb < rb:
            i += 1
        else:
            j += 1
    return out


def intersect_three_sorted_intervals(
    first: Sequence[Tuple[float, float]],
    second: Sequence[Tuple[float, float]],
    third: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    if len(first) == 1 and len(second) == 1 and len(third) == 1:
        lo, hi = first[0]
        value = second[0][0]
        if value > lo:
            lo = value
        value = second[0][1]
        if value < hi:
            hi = value
        if hi - lo <= 1e-9:
            return []
        value = third[0][0]
        if value > lo:
            lo = value
        value = third[0][1]
        if value < hi:
            hi = value
        return [(lo, hi)] if hi - lo > 1e-9 else []
    return intersect_sorted_intervals(
        intersect_sorted_intervals(first, second),
        third,
    )


SCANLINE_CELL_MM = 2.0
MATERIAL_SCANLINE_CELL_MM = 0.25


def _scanline_grid(loop: Loop) -> Dict[int, List[Segment]]:
    loop = closed_loop_geometry(loop)
    if loop.scanline_grid is not None:
        return loop.scanline_grid
    grid: Dict[int, List[Segment]] = {}
    inv = 1.0 / SCANLINE_CELL_MM
    for seg in loop.segments:
        y0 = min(seg.y0, seg.y1)
        y1 = max(seg.y0, seg.y1)
        if abs(seg.dy) <= 1e-12:
            continue
        a = math.floor(y0 * inv)
        b = math.floor(y1 * inv)
        for gy in range(a, b + 1):
            grid.setdefault(gy, []).append(seg)
    loop.scanline_grid = grid
    return grid


def scanline_intervals(loop: Loop, y: float) -> List[Tuple[float, float]]:
    loop = closed_loop_geometry(loop)
    if y < loop.y_min or y > loop.y_max:
        return []
    key = y
    if loop.scanline_cache is not None:
        cached = loop.scanline_cache.get(key)
        if cached is not None:
            return cached
    segments = loop.segments
    if len(segments) <= 32:
        candidates = segments
    else:
        grid = loop.scanline_grid
        if grid is None:
            grid = _scanline_grid(loop)
        candidates = grid.get(math.floor(y / SCANLINE_CELL_MM), ())
    xs = [
        seg.x0 + seg.dx * ((y - seg.y0) / seg.dy)
        for seg in candidates
        if (seg.y0 > y) != (seg.y1 > y)
    ]
    crossing_count = len(xs)
    if crossing_count < 2:
        intervals: List[Tuple[float, float]] = []
    elif crossing_count == 2:
        first_x, second_x = xs
        if first_x > second_x:
            first_x, second_x = second_x, first_x
        intervals = [(first_x, second_x)] if second_x - first_x > 1e-9 else []
    else:
        xs.sort()
        intervals = [
            (xs[i], xs[i + 1])
            for i in range(0, len(xs) - 1, 2)
            if xs[i + 1] - xs[i] > 1e-9
        ]
    if loop.scanline_cache is None:
        loop.scanline_cache = {}
    loop.scanline_cache[key] = intervals
    return intervals


def scanline_material_intervals(loops: Sequence[Loop], y: float) -> List[Tuple[float, float]]:
    if len(loops) == 1:
        return scanline_intervals(loops[0], y)
    events: List[Tuple[float, int]] = []
    for lp in loops:
        for a, b in scanline_intervals(lp, y):
            events.append((a, 1))
            events.append((b, -1))
    if not events:
        return []
    events.sort()
    out: List[Tuple[float, float]] = []
    depth = 0
    start: Optional[float] = None
    i = 0
    while i < len(events):
        x = events[i][0]
        delta = 0
        while i < len(events) and abs(events[i][0] - x) <= 1e-9:
            delta += events[i][1]
            i += 1
        was_inside = depth % 2 == 1
        depth += delta
        is_inside = depth % 2 == 1
        if not was_inside and is_inside:
            start = x
        elif was_inside and not is_inside and start is not None:
            if x - start > 1e-9:
                out.append((start, x))
            start = None
    return out


def scanline_layered_material_intervals(loops: Sequence[Loop], y: float) -> List[Tuple[float, float]]:
    if len(loops) == 1:
        return scanline_intervals(loops[0], y)
    by_layer: Dict[int, List[Loop]] = {}
    for lp in loops:
        by_layer.setdefault(lp.layer_index, []).append(lp)
    intervals: List[Tuple[float, float]] = []
    for layer_loops in by_layer.values():
        intervals.extend(scanline_material_intervals(layer_loops, y))
    return merge_intervals(intervals)


class LoopYIndex:
    def __init__(self, loops: Sequence[Loop]) -> None:
        self.grid: Dict[int, List[Loop]] = {}
        inv = 1.0 / SCANLINE_CELL_MM
        for loop in loops:
            geo = closed_loop_geometry(loop)
            for gy in range(math.floor(geo.y_min * inv), math.floor(geo.y_max * inv) + 1):
                self.grid.setdefault(gy, []).append(loop)

    def at(self, y: float) -> Sequence[Loop]:
        return self.grid.get(math.floor(y / SCANLINE_CELL_MM), ())


class LayerMaterialScanlineIndex:
    def __init__(self, loops: Sequence[Loop]) -> None:
        grouped_grid: Dict[int, Dict[int, List[Segment]]] = {}
        inv = 1.0 / MATERIAL_SCANLINE_CELL_MM
        for loop_index, loop in enumerate(loops):
            geo = closed_loop_geometry(loop)
            for seg in geo.segments:
                if abs(seg.dy) <= 1e-12:
                    continue
                y0 = min(seg.y0, seg.y1)
                y1 = max(seg.y0, seg.y1)
                for gy in range(math.floor(y0 * inv), math.ceil(y1 * inv)):
                    grouped_grid.setdefault(gy, {}).setdefault(loop_index, []).append(seg)
        self.grid: Dict[int, List[List[Segment]]] = {
            gy: list(by_loop.values())
            for gy, by_loop in grouped_grid.items()
        }

    def intervals(self, y: float) -> List[Tuple[float, float]]:
        segment_groups = self.grid.get(math.floor(y / MATERIAL_SCANLINE_CELL_MM), ())
        if len(segment_groups) == 1:
            segments = segment_groups[0]
            if len(segments) == 2:
                first, second = segments
                if (first.y0 > y) == (first.y1 > y):
                    return []
                if (second.y0 > y) == (second.y1 > y):
                    return []
                t = (y - first.y0) / first.dy
                a = first.x0 + first.dx * t
                t = (y - second.y0) / second.dy
                b = second.x0 + second.dx * t
                if a > b:
                    a, b = b, a
                return [(a, b)] if b - a > 1e-9 else []
            xs = [
                seg.x0 + seg.dx * ((y - seg.y0) / seg.dy)
                for seg in segments
                if (seg.y0 > y) != (seg.y1 > y)
            ]
            crossing_count = len(xs)
            if crossing_count < 2:
                return []
            if crossing_count == 2:
                a, b = xs
                if a > b:
                    a, b = b, a
                return [(a, b)] if b - a > 1e-9 else []
            xs.sort()
            return [
                (xs[i], xs[i + 1])
                for i in range(0, len(xs) - 1, 2)
                if xs[i + 1] - xs[i] > 1e-9
            ]
        endpoints: List[float] = []
        for segments in segment_groups:
            if len(segments) == 2:
                first, second = segments
                if (first.y0 > y) == (first.y1 > y):
                    continue
                if (second.y0 > y) == (second.y1 > y):
                    continue
                t = (y - first.y0) / first.dy
                a = first.x0 + first.dx * t
                t = (y - second.y0) / second.dy
                b = second.x0 + second.dx * t
                if a > b:
                    a, b = b, a
                if b - a > 1e-9:
                    endpoints.append(a)
                    endpoints.append(b)
                continue
            xs = [
                seg.x0 + seg.dx * ((y - seg.y0) / seg.dy)
                for seg in segments
                if (seg.y0 > y) != (seg.y1 > y)
            ]
            crossing_count = len(xs)
            if crossing_count == 2:
                a, b = xs
                if a > b:
                    a, b = b, a
                if b - a > 1e-9:
                    endpoints.append(a)
                    endpoints.append(b)
            elif crossing_count > 2:
                xs.sort()
                for i in range(0, len(xs) - 1, 2):
                    if xs[i + 1] - xs[i] > 1e-9:
                        endpoints.append(xs[i])
                        endpoints.append(xs[i + 1])
        if not endpoints:
            return []
        if len(endpoints) == 2:
            return [(endpoints[0], endpoints[1])]
        endpoints.sort()
        out: List[Tuple[float, float]] = []
        inside = False
        start: Optional[float] = None
        i = 0
        endpoint_count = len(endpoints)
        while i < endpoint_count:
            x = endpoints[i]
            next_i = i + 1
            while next_i < endpoint_count and endpoints[next_i] - x <= 1e-9:
                next_i += 1
            if (next_i - i) % 2 == 1:
                if inside:
                    if start is not None and x - start > 1e-9:
                        out.append((start, x))
                    start = None
                else:
                    start = x
                inside = not inside
            i = next_i
        return out


class LayeredMaterialIndex:
    def __init__(self, loops: Sequence[Loop]) -> None:
        by_layer: Dict[int, List[Loop]] = {}
        for loop in loops:
            by_layer.setdefault(loop.layer_index, []).append(loop)
        self.layer_indexes = [
            LayerMaterialScanlineIndex(layer_loops)
            for layer_loops in by_layer.values()
        ]
        self.single_index = self.layer_indexes[0] if len(self.layer_indexes) == 1 else None

    def intervals(self, y: float) -> List[Tuple[float, float]]:
        if self.single_index is not None:
            return self.single_index.intervals(y)
        intervals: List[Tuple[float, float]] = []
        for index in self.layer_indexes:
            intervals.extend(index.intervals(y))
        return merge_intervals(intervals)

def loop_overhang_arc_mm(
    loop: Loop, prev_loop: Optional[Loop], layer_height: float, spacing: float,
    thresh_deg: float, offset_scale: float = 1.0
) -> float:
    """悬垂角低于 thresh_deg 的采样弧长（mm）。约定：0 = 垂直墙，
    正 = 内缩（有支撑的正坡），负 = 外悬（悬垂）。方向用"点是否在
    前层多边形内"判定——质心半径比较对非凸轮廓（凹区、卷曲特征）
    会大面积误判。offset_scale：XY 插值开启时螺旋下半程已把路径拉回
    前层轮廓一半，等效无支撑横向偏移减半（传 0.5）。"""
    if prev_loop is None or layer_height <= 0.02:
        return 0.0
    loop = closed_loop_geometry(loop)
    step = max(spacing, 0.5)
    arc = 0.0
    traveled = 0.0
    for seg in loop.segments:
        traveled += seg.length
        if traveled < step:
            continue
        px, py = seg.x1, seg.y1
        qx, qy = nearest_on_loop((px, py), prev_loop)
        d = math.hypot(px - qx, py - qy) * offset_scale
        if d > 1e-6:
            angle = math.degrees(math.atan2(d, layer_height))
            signed = angle if point_in_loop(px, py, prev_loop) else -angle
            if signed < thresh_deg:
                arc += traveled
        traveled = 0.0
    return arc


Z_CLUSTER_TOL = 0.02  # 同一物理层内不同对象的 z 微差容忍（mm）


def cluster_z_levels(loops: Sequence[Loop]) -> Tuple[List[float], Dict[int, int]]:
    """把回路 z 聚类为物理层级。

    变高层/多对象模型中，同一物理层的不同岛屿 z 可能相差几微米
    （各对象独立规划），按精确 z 分层会把它们拆成"相邻两层"，导致
    邻层配对找不到真实邻居、层高被算成微米级。相邻 z 差 <= 容忍值
    时并入同一层级。返回 (每级代表 z 列表, id(loop)->层级序号)。"""
    zs = sorted({round(lp.z, 6) for lp in loops})
    levels: List[List[float]] = []
    for zv in zs:
        if levels and zv - levels[-1][-1] <= Z_CLUSTER_TOL:
            levels[-1].append(zv)
        else:
            levels.append([zv])
    z_to_level = {}
    reps = []
    for li, group in enumerate(levels):
        reps.append(group[-1])  # 代表 z 取组内最大（主体大轮廓通常在整数层高上）
        for zv in group:
            z_to_level[zv] = li
    loop_level = {id(lp): z_to_level[round(lp.z, 6)] for lp in loops}
    return reps, loop_level


MAX_NEIGHBOR_DZ = 0.62  # 邻层搜索的最大 z 距离（覆盖常见最大层高 + 余量）
NEIGHBOR_MATCH_MM = 2.0  # 邻层墙面中位距离阈值：同一连通体的上下层墙面必然贴近


def match_neighbor_loops(loops: Sequence[Loop]) -> Tuple[Dict[int, Loop], Dict[int, Loop]]:
    """以连通体为单位为每个回路配对上/下层邻居。

    匹配判据是**墙面贴近度**：在目标回路上均匀采样，取各采样点到候选
    回路的最近距离的中位数，选中位距离最小的候选，且须低于阈值。
    相比质心+周长比护栏，这对轮廓分裂/合并（一层内一个轮廓变两个）
    也成立——分裂瞬间质心跳变、周长突变，但外侧墙面依然逐层贴合；
    而不同连通体（跨岛）的墙面距离远超阈值，仍被正确拒绝。

    不同对象的 z 网格可能互不对齐（独立分层/微米偏差/局部变层高），
    对每个回路沿 z 向下/向上逐层级搜索直到找到匹配或超出最大层高。"""
    reps, loop_level = cluster_z_levels(loops)
    by_level: Dict[int, List[Loop]] = {}
    bbox: Dict[int, Tuple[float, float, float, float]] = {}
    samples: Dict[int, List[Tuple[float, float]]] = {}
    for lp in loops:
        by_level.setdefault(loop_level[id(lp)], []).append(lp)
        geo = closed_loop_geometry(lp)
        xs0 = [seg.x1 for seg in geo.segments]
        ys0 = [seg.y1 for seg in geo.segments]
        bbox[id(lp)] = (min(xs0), min(ys0), max(xs0), max(ys0))
        n = len(geo.segments)
        stride = max(1, n // 12)
        samples[id(lp)] = [(geo.segments[k].x1, geo.segments[k].y1) for k in range(0, n, stride)]

    def spatial_match(target: Loop, candidates: List[Loop]) -> Optional[Loop]:
        tx0, ty0, tx1, ty1 = bbox[id(target)]
        pts = samples[id(target)]
        best, best_med = None, float("inf")
        for c in candidates:
            cx0, cy0, cx1, cy1 = bbox[id(c)]
            # 包围盒间隙预筛：超过阈值不可能匹配
            gap = max(cx0 - tx1, tx0 - cx1, cy0 - ty1, ty0 - cy1)
            if gap > NEIGHBOR_MATCH_MM:
                continue
            ds = []
            for px, py in pts:
                qx, qy = nearest_on_loop((px, py), c)
                ds.append(math.hypot(qx - px, qy - py))
            ds.sort()
            med = ds[len(ds) // 2]
            if med < best_med:
                best_med = med
                best = c
        return best if best_med <= NEIGHBOR_MATCH_MM else None

    n_levels = len(reps)
    prev_map: Dict[int, Loop] = {}
    next_map: Dict[int, Loop] = {}
    for lp in loops:
        i = loop_level[id(lp)]
        j = i - 1
        while j >= 0 and lp.z - reps[j] <= MAX_NEIGHBOR_DZ:
            m = spatial_match(lp, by_level.get(j, []))
            if m is not None:
                prev_map[id(lp)] = m
                break
            j -= 1
        j = i + 1
        while j < n_levels and reps[j] - lp.z <= MAX_NEIGHBOR_DZ:
            m = spatial_match(lp, by_level.get(j, []))
            if m is not None:
                next_map[id(lp)] = m
                break
            j += 1

    # 合并/分裂层：一层里两个轮廓在上一层合成一个（或反过来）时，
    # 单一最佳匹配让合并层回路只认得其中一个下层轮廓，另一半区域的
    # 顶点找不到正下方的墙，插值被错误拉向远处的那个轮廓（"手臂被
    # 身体吸引"），动态超分还会把这个假偏移当成真斜坡放大倍率。
    # 以反向最佳匹配为归组判据：c 的最佳上邻是 t 且与 t 的最佳下邻
    # 同层级 ⟹ c 与 t 同属一个连通体，把这些邻居的段列表拼成组合
    # 回路（只用于最近点/内含查询，段各自独立，拼接不产生幻影边）。
    rev_next: Dict[int, List[Loop]] = {}
    rev_prev: Dict[int, List[Loop]] = {}
    for lp in loops:
        m = next_map.get(id(lp))
        if m is not None:
            rev_next.setdefault(id(m), []).append(lp)
        m = prev_map.get(id(lp))
        if m is not None:
            rev_prev.setdefault(id(m), []).append(lp)

    def combine(primary: Loop, others: List[Loop]) -> Loop:
        extra = [o for o in others
                 if o is not primary and loop_level[id(o)] == loop_level[id(primary)]]
        if not extra:
            return primary
        segs = list(primary.segments)
        for o in extra:
            segs.extend(o.segments)
        merged = Loop(layer_index=primary.layer_index, z=primary.z, segments=segs,
                      end_insert_index=primary.end_insert_index, has_arc=primary.has_arc)
        merged.build_cum()
        return merged

    prev_out: Dict[int, Loop] = {}
    next_out: Dict[int, Loop] = {}
    for lp in loops:
        best = prev_map.get(id(lp))
        if best is not None:
            prev_out[id(lp)] = combine(best, rev_next.get(id(lp), []))
        best = next_map.get(id(lp))
        if best is not None:
            next_out[id(lp)] = combine(best, rev_prev.get(id(lp), []))
    return prev_out, next_out


def normal_offsets_toward(
    loop: Loop,
    other: Optional[Loop],
    layer_height: float,
) -> Optional[List[Tuple[float, float]]]:
    if other is None:
        return None
    loop = closed_loop_geometry(loop)
    if loop.offset_cache is None:
        loop.offset_cache = {}
    key = (id(other), int(round(max(layer_height, 0.0) * 10000)))
    cached = loop.offset_cache.get(key)
    if cached is not None:
        return cached

    verts = loop_vertices_xy(loop)
    n_verts = len(verts)
    normals = loop_vertex_normals(loop)

    raw: List[Tuple[float, float, float]] = []
    limit_h = max(layer_height, 0.05)
    for i in range(n_verts):
        vx, vy = verts[i]
        qx, qy = nearest_on_loop((vx, vy), other)
        nx_, ny_ = normals[i]
        d_full = math.hypot(qx - vx, qy - vy)
        if d_full > 2.0 * limit_h:
            t_comp = abs((qx - vx) * -ny_ + (qy - vy) * nx_)
            if t_comp > 0.85 * d_full:
                raw.append((0.0, 0.0, 0.0))
                continue
        d = (qx - vx) * nx_ + (qy - vy) * ny_
        raw.append((nx_ * d, ny_ * d, abs(d)))

    w = max(2, min(20, n_verts // 8))
    clamped: List[Tuple[float, float]] = []
    abs_floor = 3.0 * limit_h
    mags = [r[2] for r in raw]
    for i in range(n_verts):
        ox, oy, m = raw[i]
        if m <= abs_floor:
            clamped.append((ox, oy))
            continue
        neigh = sorted(mags[(i + k) % n_verts] for k in range(-w, w + 1) if k != 0)
        local_med = neigh[len(neigh) // 2]
        limit = max(abs_floor, 3.0 * local_med)
        clamped.append((ox, oy) if m <= limit else (0.0, 0.0))

    avg_seg = loop.total_length / max(1, n_verts)
    sw = max(1, min(n_verts // 4, int(round(0.5 / max(avg_seg, 1e-6)))))
    n_k = 2 * sw + 1
    run_x = sum(clamped[k % n_verts][0] for k in range(-sw, sw + 1))
    run_y = sum(clamped[k % n_verts][1] for k in range(-sw, sw + 1))
    out: List[Tuple[float, float]] = []
    for i in range(n_verts):
        out.append((run_x / n_k, run_y / n_k))
        drop = clamped[(i - sw) % n_verts]
        add = clamped[(i + sw + 1) % n_verts]
        run_x += add[0] - drop[0]
        run_y += add[1] - drop[1]
    loop.offset_cache[key] = out
    return out


def offset_loop_toward(
    loop: Loop,
    other: Optional[Loop],
    w: float,
    clamp_mm: float = 0.6,
    offsets: Optional[List[Tuple[float, float]]] = None,
) -> Loop:
    """把回路顶点沿局部法线向 other 轮廓偏移 w 比例，返回新回路。

    与 build_spiral 的 XY 插值同一公式（法向投影 + 离群钳制），用于让
    二次整形/排压圈跟随插值后的顶面轮廓，而不是骑在原始轮廓上悬空刮边。"""
    loop = closed_loop_geometry(loop)
    if (other is None and offsets is None) or w == 0.0:
        return loop
    verts = loop_vertices_xy(loop)
    n = len(verts)
    normals = loop_vertex_normals(loop) if offsets is None else []

    def raw_offset(i: int) -> Tuple[float, float, float]:
        nx_, ny_ = normals[i]
        if nx_ == 0.0 and ny_ == 0.0:
            return 0.0, 0.0, 0.0
        vx, vy = verts[i]
        qx, qy = nearest_on_loop((vx, vy), other)
        d_full = math.hypot(qx - vx, qy - vy)
        # 伪对应检验（切向占比），同 build_spiral.normal_offsets
        if d_full > 2.0 * clamp_mm / 3.0:
            t_comp = abs((qx - vx) * -ny_ + (qy - vy) * nx_)
            if t_comp > 0.85 * d_full:
                return 0.0, 0.0, 0.0
        d = (qx - vx) * nx_ + (qy - vy) * ny_
        return nx_ * d, ny_ * d, abs(d)

    if offsets is not None:
        new_pts = [(verts[i][0] + offsets[i][0] * w, verts[i][1] + offsets[i][1] * w) for i in range(n)]
    else:
        raw = [raw_offset(i) for i in range(n)]
        mags = sorted(r[2] for r in raw)
        median = mags[len(mags) // 2]
        limit = max(clamp_mm, 4.0 * median)
        new_pts = [
            (verts[i][0] + raw[i][0] * w, verts[i][1] + raw[i][1] * w) if raw[i][2] <= limit else verts[i]
            for i in range(n)
        ]
    segments: List[Segment] = []
    prev_pt = new_pts[-1]
    for i, seg in enumerate(loop.segments):
        pt = new_pts[i]
        length = math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1])
        safe_length = max(length, 1e-6)
        de = seg.de * (safe_length / seg.length) if seg.length > 1e-9 else seg.de
        segments.append(Segment(
            x0=prev_pt[0], y0=prev_pt[1], z0=seg.z0,
            x1=pt[0], y1=pt[1], z1=seg.z1,
            de=de, length=safe_length, line_index=seg.line_index,
        ))
        prev_pt = pt
    out = Loop(layer_index=loop.layer_index, z=loop.z, segments=segments,
               end_insert_index=loop.end_insert_index, has_arc=loop.has_arc)
    out.build_cum()
    return out


def build_spiral(
    loop: Loop,
    config: Dict[str, Any],
    layer_height: float,
    wall_feed: float,
    flatten: bool,
    prev_loop: Optional[Loop] = None,
    next_loop: Optional[Loop] = None,
) -> List[Ins]:
    """亚层螺旋（旋转花瓶式接缝消除）。

    整层外墙替换为 x 圈连续螺旋：从 z-h 起 Z 沿弧长匀速爬升，每圈升 h/x，
    x 圈后到达本层高度。每圈流量为主体流量的 1/x（对应 h/x 的沉积厚度）。
    首圈流量按间隙线性渐升（下方是平的前层顶面），中间各圈完全无接缝；
    收尾在本层高度补一圈互补流量（1 - s/L）填平顶部楔形，全程流量与 Z
    连续，无任何压力突变点。最后可选零流量空转一圈压平。总挤出量精确
    等于 主体流量 × 原外墙挤出量。

    亚层级精度优化（XY 插值）：把本层轮廓视为层片中点的表面采样，子圈
    在高度分数 t 处的 XY 向相邻层轮廓插值——下半程向前一层（权重最大
    0.5），上半程向下一层——用三层状态重建表面斜率，子圈不再垂直堆叠。"""
    loop = closed_loop_geometry(loop)
    base = fnum(config, "primary_flow_scale")
    x = max(1, int(fnum(config, "seam_superres_x")))
    interp = bool(config.get("spiral_xy_interp_enabled", True))
    comp_len = max(0.0, fnum(config, "seam_comp_length_mm"))
    comp_scale = max(0.0, fnum(config, "seam_comp_scale"))
    total = loop.total_length
    h = layer_height
    z_top = loop.z

    # 每个顶点在相邻层轮廓上的对应偏移：取最近点连线在局部法线方向上的
    # 分量（纯法向偏移）。直接用最近点会沿前层折线切向滑移，把弦差噪声
    # 带进轮廓；法向投影只保留真实的表面斜率信息
    verts = loop_vertices_xy(loop)
    n_verts = len(verts)

    def normal_at(i: int) -> Tuple[float, float]:
        a = verts[(i - 1) % n_verts]
        b = verts[(i + 1) % n_verts]
        tx, ty = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(tx, ty)
        if norm <= 1e-9:
            return 0.0, 0.0
        return ty / norm, -tx / norm

    def normal_offsets(other: Optional[Loop]) -> Optional[List[Tuple[float, float]]]:
        if other is None:
            return None
        raw: List[Tuple[float, float, float]] = []  # (nx*d, ny*d, |d|)
        for i in range(n_verts):
            vx, vy = verts[i]
            qx, qy = nearest_on_loop((vx, vy), other)
            nx_, ny_ = normal_at(i)
            d_full = math.hypot(qx - vx, qy - vy)
            # 伪对应检验（切向占比）：真实斜面的邻层对应点应大致沿本地
            # 法线方向（趾尖端头等曲率大处 ~0.5-0.7）；相邻层轮廓后退的
            # 边缘（卷叶尖、缺口）匹配到的是不相关墙段，偏移几乎纯切向
            # （实测 >0.9）。切向占比超阈值判伪清零——该处没有可插值的
            # 表面，垂直堆叠才是正确形状
            if d_full > 2.0 * max(layer_height, 0.05):
                t_comp = abs((qx - vx) * -ny_ + (qy - vy) * nx_)
                if t_comp > 0.85 * d_full:
                    raw.append((0.0, 0.0, 0.0))
                    continue
            d = (qx - vx) * nx_ + (qy - vy) * ny_
            raw.append((nx_ * d, ny_ * d, abs(d)))
        # 离群钳制（局部窗口）：合法的斜面偏移沿回路是平滑的——即使
        # 整圈大部分垂直、只有顶部平坦区偏移很大（全局中位数会误杀它），
        # 局部邻域内偏移仍相近。层间轮廓失配（岛屿分叉/特征突变）表现
        # 为孤立尖峰，远超邻域水平。窗口取 ±min(20, n/8) 个顶点。
        w = max(2, min(20, n_verts // 8))
        clamped: List[Tuple[float, float]] = []
        abs_floor = 3.0 * max(layer_height, 0.05)
        mags = [r[2] for r in raw]
        for i in range(n_verts):
            ox, oy, m = raw[i]
            # 快速路径：不超过绝对下限的点必然通过，无需算局部中位数
            if m <= abs_floor:
                clamped.append((ox, oy))
                continue
            neigh = sorted(
                mags[(i + k) % n_verts] for k in range(-w, w + 1) if k != 0
            )
            local_med = neigh[len(neigh) // 2]
            limit = max(abs_floor, 3.0 * local_med)
            clamped.append((ox, oy) if m <= limit else (0.0, 0.0))
        # 平滑（弧长 ~1mm 箱式滤波）：曲率大/轮廓切向滑移处最近点匹配
        # 会在相邻顶点间来回跳（偏移场锯齿），直接用会让子圈路径抖成
        # 折线。真实表面斜率沿弧长是缓变的，低通后只留斜率信息
        avg_seg = loop.total_length / max(1, n_verts)
        sw = max(1, min(n_verts // 4, int(round(0.5 / max(avg_seg, 1e-6)))))
        # 滑动窗口累加 O(n)（箱式滤波等价实现）
        n_k = 2 * sw + 1
        run_x = sum(clamped[k % n_verts][0] for k in range(-sw, sw + 1))
        run_y = sum(clamped[k % n_verts][1] for k in range(-sw, sw + 1))
        out: List[Tuple[float, float]] = []
        for i in range(n_verts):
            out.append((run_x / n_k, run_y / n_k))
            drop = clamped[(i - sw) % n_verts]
            add = clamped[(i + sw + 1) % n_verts]
            run_x += add[0] - drop[0]
            run_y += add[1] - drop[1]
        return out

    prev_off = normal_offsets_toward(loop, prev_loop, layer_height) if interp else None
    next_off = normal_offsets_toward(loop, next_loop, layer_height) if interp else None

    # 动态超分：斜面处子圈横向错开量 = 层间轮廓偏移 / x。若相邻子圈
    # 横向间距超过 (1 - overlap_frac) * 线宽，圈与圈之间盖不住会留缝
    # （顶部缓坡正是偏移最大的地方）。开启时自动把 x 提升到刚好满足
    # 重叠要求。线宽由原始外墙挤出量反推（E 为耗材长度，1.75mm 耗材）。
    if bool(config.get("dynamic_superres_enabled", True)) and interp and h > 0.02 and total > 0:
        overlap = max(0.0, min(0.9, fnum(config, "spiral_overlap_frac")))
        loop_ext = sum(seg.de for seg in loop.segments)
        fil_area = math.pi * (1.75 / 2.0) ** 2
        inferred_width = (loop_ext / total) * fil_area / h
        configured_width = max(0.05, fnum(config, "path_width_mm"))
        line_width = min(inferred_width, configured_width)
        if line_width > 1e-6:
            # 用平滑+钳制后的偏移场取最大值：离群已在 normal_offsets 清掉，
            # 最大值代表真实的最平坦区。分位数会漏掉只占周长一小段的
            # 局部平坦带（上下层在该带内接不上）
            span = 0.0
            for i in range(n_verts):
                if next_off is None and prev_off is not None:
                    sp = 0.5 * math.hypot(*prev_off[i])
                else:
                    sp = 0.0
                    if prev_off is not None:
                        sp += 0.5 * math.hypot(*prev_off[i])
                    if next_off is not None:
                        sp += 0.5 * math.hypot(*next_off[i])
                if sp > span:
                    span = sp
            max_step = (1.0 - overlap) * line_width
            if max_step > 1e-6:
                x_needed = math.ceil(span / max_step)
                x_cap = max(x, int(fnum(config, "superres_max_x")))
                x = min(x_cap, max(x, x_needed))
    per_rev = base / x

    def vertex_at(i: int, t: float) -> Tuple[float, float]:
        vx, vy = verts[i]
        if not interp:
            return vx, vy
        if next_off is None and prev_off is not None:
            # 最顶层：无下一层轮廓，把下半程拉伸到整层——t=0 从与前层
            # 中点（上一层螺旋的收尾位置）出发，t=1 到达自身轮廓，
            # 全层匀速渐变，与上一层连续衔接
            w = (t - 1.0) * 0.5
            ox, oy = prev_off[i]
            return vx + ox * -w, vy + oy * -w
        w = max(-0.5, min(0.5, t - 0.5))
        if w < 0 and prev_off is not None:
            ox, oy = prev_off[i]
            return vx + ox * -w, vy + oy * -w
        if w > 0 and next_off is not None:
            ox, oy = next_off[i]
            return vx + ox * w, vy + oy * w
        return vx, vy

    angle_speed_enabled = bool(config.get("spiral_angle_speed_enabled", False))
    angle_speed_profile = (
        configured_spiral_angle_speed_profile(config)
        if angle_speed_enabled
        else DEFAULT_SPIRAL_ANGLE_SPEED_POINTS
    )
    revolution_dt = 1.0 / x

    def angle_limited_feed(i: int, t_start: float, t_end: float) -> float:
        """Use adjacent final sub-layer tracks to measure the local surface angle."""
        prev_i = (i - 1) % n_verts

        def shifted_center(shift: float) -> Tuple[float, float, float]:
            shifted_start = max(0.0, min(1.0, t_start + shift))
            shifted_end = max(0.0, min(1.0, t_end + shift))
            ax, ay = vertex_at(prev_i, shifted_start)
            bx, by = vertex_at(i, shifted_end)
            return (
                0.5 * (ax + bx),
                0.5 * (ay + by),
                0.5 * h * (shifted_start + shifted_end),
            )

        lower = shifted_center(-revolution_dt)
        upper = shifted_center(revolution_dt)
        dxy = math.hypot(upper[0] - lower[0], upper[1] - lower[1])
        dz = abs(upper[2] - lower[2])
        angle = 90.0 if dxy <= 1e-12 else math.degrees(math.atan2(dz, dxy))
        return wall_feed * spiral_angle_speed_multiplier(angle, angle_speed_profile)

    ins: List[Ins] = [Ins(kind="comment", text=f"; BOWP spiral start x{x}\n")]
    # 接缝起点补偿（再分配式）：回填后的腔压峰值把后续的料提前挤出形成
    # 突起，突起消耗压力后紧跟一段欠压。补偿按**整圈满流量**为基准在开头
    # 减料（首圈渐升流量本身近零，按它减没有实际力度），下限钳到零；
    # 实际减掉的料原量摊回随后 payback 段（三角权重递减）填补欠压凹陷
    # ——总挤出量守恒，不产生接缝后净凹陷。
    comp_active = comp_len > 0 and comp_scale > 0
    payback_len = min(2.0 * comp_len, max(0.0, total - comp_len)) if comp_active else 0.0

    def comp_remove(s_mid: float, full_de: float) -> float:
        # 前置加重的二次衰减：起点减得最狠，向 comp_len 平滑归零；
        # 强度 >1 时零流量区间相应变长
        return full_de * comp_scale * (1.0 - s_mid / comp_len) ** 2

    removed_actual = 0.0
    payback_base = 0.0
    if comp_active:
        s0 = 0.0
        for seg in loop.segments:
            s_mid = s0 + seg.length / 2
            if payback_len > 0 and comp_len <= s_mid < comp_len + payback_len:
                payback_base += seg.de * per_rev * (
                    1.0 - (s_mid - comp_len) / payback_len
                )
            s0 += seg.length

    # 下探到前层顶面，从接缝点（t=0 的插值位置）开始爬升。
    # 接缝分散后起点可能离当前喷嘴位置较远：先在本层高度水平空移到
    # 起点上方，再原地下探——避免斜向下穿越已打印结构刮擦
    sx, sy = vertex_at(len(verts) - 1, 0.0)
    ins.append(Ins(kind="travel", x=sx, y=sy, z=z_top))
    ins.append(Ins(kind="travel", x=sx, y=sy, z=z_top - h))
    # 流量按实际发出的路径长度计算（线密度恒定）：插值会把子圈推到
    # 不同半径，周长随之变化；若按原轮廓段长给料，缓坡处每层会出现
    # 欠挤->正常的周期性条带
    px_, py_ = sx, sy
    for k in range(x):
        s0 = 0.0
        for i, seg in enumerate(loop.segments):
            s_mid = s0 + seg.length / 2
            s_end = s0 + seg.length
            t = (k + s_end / total) / x
            zf = z_top - h + h * t
            vx, vy = vertex_at(i, t)
            actual = math.hypot(vx - px_, vy - py_)
            de = seg.de * (actual / seg.length if seg.length > 1e-9 else 1.0) * per_rev
            if k == 0:
                # 首圈间隙从 0 线性长到 h/x，流量同比例（中点积分对线性精确）
                de *= s_mid / total
                # 接缝起点补偿：按满流量基准减料（钳到零），payback 段等量补回
                if comp_active:
                    if s_mid < comp_len:
                        cut = min(de, comp_remove(s_mid, seg.de * per_rev))
                        removed_actual += cut
                        de -= cut
                    elif payback_len > 0 and s_mid < comp_len + payback_len and payback_base > 1e-12:
                        w_pb = seg.de * per_rev * (
                            1.0 - (s_mid - comp_len) / payback_len
                        )
                        de += removed_actual * (w_pb / payback_base)
            segment_feed = (
                angle_limited_feed(i, (k + s0 / total) / x, t)
                if angle_speed_enabled
                else wall_feed
            )
            ins.append(Ins(kind="extrude", x=vx, y=vy, z=zf, de=de, f=segment_feed))
            px_, py_ = vx, vy
            s0 = s_end
    # 顶部互补填平：螺旋顶面遗留 (1 - s/L)*h/x 的楔形缺口
    ins.append(Ins(kind="comment", text="; BOWP spiral top fill\n"))
    s0 = 0.0
    for i, seg in enumerate(loop.segments):
        s_mid = s0 + seg.length / 2
        vx, vy = vertex_at(i, 1.0)
        actual = math.hypot(vx - px_, vy - py_)
        de = seg.de * (actual / seg.length if seg.length > 1e-9 else 1.0) * per_rev * (1.0 - s_mid / total)
        segment_feed = angle_limited_feed(i, 1.0, 1.0) if angle_speed_enabled else wall_feed
        ins.append(Ins(kind="extrude", x=vx, y=vy, z=z_top, de=de, f=segment_feed))
        px_, py_ = vx, vy
        s0 += seg.length
    if flatten:
        ins.append(Ins(kind="comment", text="; BOWP spiral flatten\n"))
        for i, seg in enumerate(loop.segments):
            vx, vy = vertex_at(i, 1.0)
            segment_feed = angle_limited_feed(i, 1.0, 1.0) if angle_speed_enabled else wall_feed
            ins.append(Ins(kind="extrude", x=vx, y=vy, z=z_top, de=0.0, f=segment_feed))
    ins.append(Ins(kind="comment", text="; BOWP spiral end\n"))
    return ins


def ramp_avg(a: float, b: float, up: float, down: float, total: float) -> float:
    """流量渐变系数在区间 [a,b] 上的平均值。

    系数曲线：0->up 线性升 0..1，中段 1，total-down->total 线性降 1..0。"""
    if b <= a:
        return 1.0

    def integral(t: float) -> float:
        # 系数从 0 积到 t
        t = max(0.0, min(t, total))
        up_end = min(up, total)
        area = 0.0
        if up > 0:
            u = min(t, up_end)
            area += (u * u) / (2 * up)
            if t <= up_end:
                return area
        flat_end = max(up_end, total - down)
        f = min(max(t, up_end), flat_end)
        area += f - up_end
        if t <= flat_end or down <= 0:
            return area
        d = t - flat_end
        area += d - (d * d) / (2 * down)
        return area

    return max(0.0, (integral(b) - integral(a)) / (b - a))


def build_secondary_pass(
    loop: Loop, config: Dict[str, Any], short_loop: bool,
    wall_feed: float = DEFAULT_FEED, continuous: bool = False,
) -> List[Ins]:
    loop = closed_loop_geometry(loop)
    total = loop.total_length
    if total <= 0.1:
        return []
    abs_offset = fnum(config, "secondary_start_offset_abs_mm")
    rel_offset = fnum(config, "secondary_start_offset_rel")
    offset = abs_offset if total >= abs_offset else total * rel_offset
    flow = fnum(config, "secondary_flow_scale")
    # 自适应速度：以外墙原速度为基准按倍率缩放
    speed_normal = wall_feed * max(0.05, fnum(config, "secondary_speed_scale"))
    speed_seam = wall_feed * max(0.05, fnum(config, "secondary_seam_speed_scale"))
    scarf = max(0.0, fnum(config, "scarf_length_mm"))
    mode = str(config.get("secondary_mode", "full_loop"))
    length = total if mode == "full_loop" or short_loop else min(total, fnum(config, "secondary_window_mm"))
    if continuous:
        # 螺旋衔接：从螺旋落点（回路起点）连续接打，无空移；
        # 空转圈是零流量，起步需要流量渐升避免 0 -> flow 的突变，
        # 结束前渐降到零
        offset = 0.0
        ramp_up = min(2.0, length * 0.15)
        ramp_down = min(2.0, length * 0.15)
    else:
        ramp_up = ramp_down = min(2.0, length * 0.15)

    def shaped_feed(loop_pos: float) -> float:
        if continuous or scarf <= 1e-6 or abs(speed_normal - speed_seam) <= 1e-6:
            return speed_normal
        seam_dist = min(loop_pos, total - loop_pos)
        if seam_dist >= scarf:
            return speed_normal
        t = max(0.0, min(1.0, seam_dist / scarf))
        t = t * t * (3.0 - 2.0 * t)
        return speed_seam + (speed_normal - speed_seam) * t

    start_point, start_seg = point_along_loop(loop, offset)
    commands: List[Ins] = [Ins(kind="comment", text="; BOWP secondary pass start\n")]
    if not continuous:
        commands.append(Ins(kind="travel", x=start_point[0], y=start_point[1], z=loop.z))
    position = offset
    remaining = length
    seg_index = start_seg
    seg = loop.segments[seg_index]
    seg_start_dist = loop.cum[seg_index - 1] if seg_index > 0 else 0.0
    frac_into_seg = (offset % total) - seg_start_dist
    while remaining > 1e-6:
        available = seg.length - frac_into_seg
        step = min(available, remaining)
        if step <= 1e-9:
            seg_index = (seg_index + 1) % len(loop.segments)
            seg = loop.segments[seg_index]
            frac_into_seg = 0.0
            continue
        end_frac = (frac_into_seg + step) / seg.length
        target = (
            seg.x0 + (seg.x1 - seg.x0) * end_frac,
            seg.y0 + (seg.y1 - seg.y0) * end_frac,
            loop.z,
        )
        de = seg.de * (step / seg.length) * flow
        # 起步/收尾渐变：对步区间积分（而非中点采样），长段跨越 ramp 时同样精确
        a = position - offset
        b = a + step
        de *= ramp_avg(a, b, ramp_up, ramp_down, length)
        loop_pos = (position + step * 0.5) % total
        # 螺旋模式接缝已消除，不存在"接缝段"——全程恒速，避免一圈内
        # 两次速度突变正好落在原接缝点上造成堆料
        feed = shaped_feed(loop_pos)
        commands.append(Ins(kind="extrude", x=target[0], y=target[1], z=loop.z, de=de, f=feed))
        position += step
        remaining -= step
        frac_into_seg += step
        if seg.length - frac_into_seg <= 1e-9:
            seg_index = (seg_index + 1) % len(loop.segments)
            seg = loop.segments[seg_index]
            frac_into_seg = 0.0
    commands.append(Ins(kind="comment", text="; BOWP secondary pass end\n"))
    return commands


def build_script_ironing(
    loop: Loop,
    config: Dict[str, Any],
    layer_height: float,
    next_loop: Optional[Loop],
    reach_entry: Optional[Tuple[Loop, List[float], List[float]]],
    blockers: Sequence[Loop] = (),
    next_covers: Sequence[Loop] = (),
    current_material: Sequence[Loop] = (),
    next_cover_index: Optional[LayeredMaterialIndex] = None,
    current_material_index: Optional[LayeredMaterialIndex] = None,
) -> List[Ins]:
    """后处理内置顶面熨烫：在本层外墙内、未被本层螺旋/高层路径覆盖处画平行线。

    判定用采样点：点必须在当前闭合外墙内；若下一层轮廓包含该点，说明高一层
    还会打印覆盖，不熨烫；若点落在螺旋实际到达半径内，说明亚层螺旋已经覆盖，
    也不熨烫。剩余区域就是裸露顶面/无法被逐层收敛外墙覆盖的区域。"""
    if not loop_is_closed(loop) or loop.total_length <= 1.0 or layer_height <= 0.02:
        return []
    if loop_z_span(loop) > max(0.02, layer_height * 0.2):
        return []
    loop = closed_loop_geometry(loop)
    width = max(0.05, fnum(config, "path_width_mm"))
    overlap = max(0.0, min(0.9, fnum(config, "spiral_overlap_frac")))
    spacing = max(0.05, width * (1.0 - overlap))
    flow = max(0.0, fnum(config, "script_ironing_flow"))
    if flow <= 1e-9:
        return []
    feed = max(1.0, fnum(config, "script_ironing_speed_mm_s")) * 60.0
    h = layer_height
    fil_area = math.pi * (1.75 / 2.0) ** 2
    line_density = flow * width * h / fil_area
    xs = [p for seg in loop.segments for p in (seg.x0, seg.x1)]
    ys = [p for seg in loop.segments for p in (seg.y0, seg.y1)]
    if not xs or not ys:
        return []
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    margin = width * 0.5
    half_spacing = spacing * 0.5
    probe = min(0.0001, spacing * 0.001)
    scan_y_limit = max_y - spacing * 0.25
    span_bin_size = max(spacing, 1e-6)
    nearby_row_dy = spacing * 0.75
    cover_loops = list(next_covers) if next_covers else ([next_loop] if next_loop is not None else [])
    blocker_index = LoopYIndex(blockers)
    cover_material_index = next_cover_index or (
        LayeredMaterialIndex(cover_loops) if cover_loops else None
    )
    current_index = current_material_index or (
        LayeredMaterialIndex(current_material) if current_material else None
    )
    cover_material_intervals = None
    if cover_material_index is not None:
        cover_material_intervals = (
            cover_material_index.single_index.intervals
            if cover_material_index.single_index is not None
            else cover_material_index.intervals
        )
    current_material_intervals = None
    if current_index is not None:
        current_material_intervals = (
            current_index.single_index.intervals
            if current_index.single_index is not None
            else current_index.intervals
        )
    cover_ref: Optional[Loop] = None
    cover_radii: List[float] = []
    if reach_entry is not None:
        cover_ref, cover_radii, _ = reach_entry
        if max(cover_radii, default=0.0) < 0.05:
            cover_ref = None
            cover_radii = []

    def spiral_covered(px: float, py: float) -> bool:
        if cover_ref is None:
            return False
        _, _, idx, d2 = nearest_on_loop_with_index((px, py), cover_ref)
        n = len(cover_radii)
        if n == 0:
            return False
        local = max(cover_radii[(idx - 1) % n], cover_radii[idx], cover_radii[(idx + 1) % n])
        cover_radius = min(local + margin, width + margin)
        return local >= 0.05 and math.sqrt(d2) <= cover_radius

    def uncovered_spans(a: float, b: float, yy: float) -> List[Tuple[float, float]]:
        a += margin
        b -= margin
        if b - a < width:
            return []
        if reach_entry is None:
            return [(a, b)]
        out: List[Tuple[float, float]] = []
        x = a + half_spacing
        in_run = False
        run_start = a
        last_good = a
        while x <= b:
            bare = not spiral_covered(x, yy)
            if bare and not in_run:
                in_run = True
                run_start = max(a, x - half_spacing)
            if bare:
                last_good = min(b, x + half_spacing)
            elif in_run:
                if last_good - run_start >= width:
                    out.append((run_start, last_good))
                in_run = False
            x += spacing
        if in_run and last_good - run_start >= width:
            out.append((run_start, last_good))
        return out

    spans: List[Tuple[float, float, float]] = []
    span_bins: Dict[int, List[Tuple[float, float, float]]] = {}

    def add_span(a: float, b: float, yy: float, supplemental: bool) -> None:
        key = math.floor(yy / span_bin_size)
        if supplemental:
            for ky in (key - 1, key, key + 1):
                for ea, eb, ey in span_bins.get(ky, ()):
                    if abs(ey - yy) <= nearby_row_dy and min(b, eb) - max(a, ea) >= width:
                        return
        spans.append((a, b, yy))
        span_bins.setdefault(key, []).append((a, b, yy))

    def supplemental_row_is_redundant(
        intervals: Sequence[Tuple[float, float]],
        yy: float,
    ) -> bool:
        key = math.floor(yy / span_bin_size)
        for a, b in intervals:
            a += margin
            b -= margin
            if b - a < width:
                continue
            contained = False
            for ky in (key - 1, key, key + 1):
                for ea, eb, ey in span_bins.get(ky, ()):
                    if abs(ey - yy) <= nearby_row_dy and ea <= a and eb >= b:
                        contained = True
                        break
                if contained:
                    break
            if not contained:
                return False
        return True

    def scan_at_y(y: float, supplemental: bool) -> None:
        if y < min_y or y > max_y:
            return
        intervals = intersect_three_sorted_intervals(
            scanline_intervals(loop, y),
            scanline_intervals(loop, y - probe),
            scanline_intervals(loop, y + probe),
        )
        if not intervals:
            return
        if supplemental and supplemental_row_is_redundant(intervals, y):
            return
        cover_cuts: List[Tuple[float, float]] = []
        if cover_material_intervals is not None:
            for a, b in cover_material_intervals(y):
                cover_cuts.append((a - margin, b + margin))
            if cover_cuts and not subtract_intervals(intervals, cover_cuts, True, True):
                return
            for yy in (y - probe, y + probe):
                for a, b in cover_material_intervals(yy):
                    cover_cuts.append((a - margin, b + margin))
            if cover_cuts:
                intervals = subtract_intervals(intervals, cover_cuts, True)
                if not intervals:
                    return
        if current_material_intervals is not None:
            material = intersect_three_sorted_intervals(
                current_material_intervals(y),
                current_material_intervals(y - probe),
                current_material_intervals(y + probe),
            )
            intervals = intersect_sorted_intervals(intervals, material)
            if not intervals:
                return
        cuts: List[Tuple[float, float]] = []
        for yy in (y, y - probe, y + probe):
            for blocker in blocker_index.at(yy):
                for a, b in scanline_intervals(blocker, yy):
                    cuts.append((a - margin, b + margin))
        spans_at_y = subtract_intervals(intervals, cuts, True)
        for a, b in spans_at_y:
            for ua, ub in uncovered_spans(a, b, y):
                add_span(ua, ub, y, supplemental)

    def scan_rows(start_frac: float, supplemental: bool) -> None:
        y = min_y + spacing * start_frac
        while y <= scan_y_limit:
            scan_at_y(y, supplemental)
            y += spacing

    scan_rows(0.5, False)
    if cover_loops or blockers:
        for phase in (0.125, 0.25, 0.375, 0.625, 0.75, 0.875, 1.0):
            scan_rows(phase, True)
        y_marks: Dict[float, int] = {}

        def mark_y(value: float, bit: int) -> None:
            if min_y - spacing <= value <= max_y + spacing:
                key = round(value, 5)
                y_marks[key] = y_marks.get(key, 0) | bit

        for seg in loop.segments:
            mark_y(seg.y0, 1)
            mark_y(seg.y1, 1)
        for cut_loop in cover_loops:
            for seg in cut_loop.segments:
                mark_y(seg.y0, 2)
                mark_y(seg.y1, 2)
        for blocker in blockers:
            for seg in blocker.segments:
                mark_y(seg.y0, 2)
                mark_y(seg.y1, 2)
        ordered_y = sorted(y_marks)
        added_rows = 0
        max_gap = min(width * 1.5, 0.6)
        for ya, yb in zip(ordered_y, ordered_y[1:]):
            if yb - ya <= 1e-5 or yb - ya > max_gap:
                continue
            if (y_marks[ya] & 1 and y_marks[yb] & 2) or (y_marks[ya] & 2 and y_marks[yb] & 1):
                scan_at_y((ya + yb) * 0.5, True)
                added_rows += 1
                if added_rows >= 512:
                    break

    if not spans:
        return []
    ins: List[Ins] = [Ins(kind="comment", text="; BOWP script ironing start\n")]
    left_to_right = True
    for x0, x1, yy in spans:
        # 端点内缩半线宽，避免熨烫线心骑到外墙外侧；采样确认中间仍为裸露区
        a = x0
        b = x1
        if b - a < width:
            continue
        sx, ex = (a, b) if left_to_right else (b, a)
        ins.append(Ins(kind="travel", x=sx, y=yy, z=loop.z))
        length = abs(ex - sx)
        ins.append(Ins(kind="extrude", x=ex, y=yy, z=loop.z, de=line_density * length, f=feed))
        left_to_right = not left_to_right
    if len(ins) == 1:
        return []
    ins.append(Ins(kind="comment", text="; BOWP script ironing end\n"))
    return ins


def pending_retract(infos: Sequence[LineInfo], loop: Loop, insert_index: int) -> float:
    """外墙结束到插入点之间的净挤出变化；为负表示插入时已处于回抽状态。"""
    last = loop.segments[-1].line_index
    net = 0.0
    for i in range(last + 1, min(insert_index, len(infos))):
        net += infos[i].de
    return net


def build_purge_lap(sub: Loop, retract_mm: float, wall_feed: float) -> List[Ins]:
    """终点排压圈：零流量沿轮廓再走一整圈，回抽量摊到整圈逐步完成。

    最终停止点不再有静止回抽的余压淤积——压力沿整圈释放，停止时
    已处于回抽状态。"""
    sub = closed_loop_geometry(sub)
    total = sub.total_length
    if total <= 0:
        return []
    ins: List[Ins] = [Ins(kind="comment", text="; BOWP purge lap start\n")]
    # 排压圈起点 = 回路起点；上一段（二次整形）可能停在偏移点，
    # 先空移衔接，避免带负压跨越表面
    first_seg = sub.segments[0]
    ins.append(Ins(kind="travel", x=first_seg.x0, y=first_seg.y0, z=sub.z))
    for seg in sub.segments:
        ins.append(Ins(kind="extrude", x=seg.x1, y=seg.y1, z=sub.z,
                       de=-retract_mm * seg.length / total, f=wall_feed))
    ins.append(Ins(kind="comment", text="; BOWP purge lap end\n"))
    return ins


def neutralize_trailing_retract(
    infos: Sequence[LineInfo], start_index: int, de_scale: Dict[int, float]
) -> float:
    """把外墙后紧随的原始回抽/擦拭负 E 行清零，返回原始回抽总量。

    排压圈接管回抽后，原始静止回抽必须取消，否则双重回抽。"""
    w = 0.0
    for i in range(start_index, min(start_index + 40, len(infos))):
        info = infos[i]
        marker = info.line.comment.upper() if info.line.comment else ""
        if "TYPE:" in marker or any(t in marker for t in LAYER_MARKERS):
            break
        if info.kind != "move":
            continue
        if info.de > 1e-9:
            break
        if info.de < -1e-9:
            w += -info.de
            de_scale[info.index] = 0.0
    return w


def wrap_block(commands: List[Ins], retract_amount: float, smear_mm: float = 2.0) -> List[Ins]:
    if retract_amount >= -0.01 or not commands:
        return commands
    out = list(commands)
    prime_total = -retract_amount
    retract = Ins(kind="prime", de=retract_amount, f=UNRETRACT_FEED)
    # 边走边回填：把回填量摊进起步头 smear_mm 的挤出移动里（按移动距离
    # 加权分配到各段的 de 上），避免静止一次性推回造成压力过冲淤积。
    # 结束时仍一次性重新回抽（回抽不产生淤积问题）。
    first_ext = None
    moves: List[Tuple[int, float]] = []  # (index, seg_length)
    px = py = None
    for idx, ins in enumerate(out):
        if ins.kind == "travel":
            px = ins.x if ins.x is not None else px
            py = ins.y if ins.y is not None else py
        elif ins.kind == "extrude":
            if first_ext is None:
                first_ext = idx
            length = 0.0
            if px is not None and ins.x is not None:
                length = math.hypot(ins.x - px, (ins.y or 0.0) - (py or 0.0))
            moves.append((idx, max(length, 1e-3)))
            px, py = ins.x, ins.y
            acc = sum(l for _, l in moves)
            if acc >= smear_mm:
                break
    if first_ext is None:
        # 块内没有挤出移动：退回静止回填
        out.insert(len(out) - 1, Ins(kind="prime", de=prime_total, f=UNRETRACT_FEED))
        out.insert(len(out) - 1, retract)
        return out
    smear_total = sum(l for _, l in moves)
    for idx, length in moves:
        out[idx] = Ins(kind=out[idx].kind, text=out[idx].text, x=out[idx].x, y=out[idx].y,
                       z=out[idx].z, de=out[idx].de + prime_total * (length / smear_total),
                       f=out[idx].f)
    out.insert(len(out) - 1, retract)
    return out


def render_output_stream(
    infos: Sequence[LineInfo],
    de_scale: Dict[int, float],
    replacements: Dict[int, List[Ins]],
    insertions: Dict[int, List[List[Ins]]],
    config: Dict[str, Any],
    write_line: Callable[[str], None],
) -> Tuple[float, float, float, float, float]:
    relative = False
    coord_relative = False
    e_cursor = 0.0
    rel_residual = 0.0
    x = y = z = 0.0
    feed = DEFAULT_FEED
    travel_feed = fnum(config, "travel_speed_mm_s") * 60.0
    est = TimeEstimator()
    final_path = 0.0
    final_ext = 0.0
    added_path = 0.0
    added_ext = 0.0

    def simulate(nx: float, ny: float, nz: float, f: float) -> float:
        nonlocal x, y, z, final_path
        dx, dy, dz = nx - x, ny - y, nz - z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > 0 and f > 0:
            est.move(dx, dy, dz, distance, f / 60.0)
        final_path += distance
        x, y, z = nx, ny, nz
        return distance

    def emit_e(args: Dict[str, float], de: float) -> None:
        nonlocal e_cursor, rel_residual
        if relative:
            want = de + rel_residual
            emitted = float(format_float(want))
            rel_residual = want - emitted
            args["E"] = emitted
        else:
            e_cursor += de
            args["E"] = e_cursor

    def emit_ins(ins: Ins, count_added: bool = True) -> None:
        nonlocal added_path, added_ext, final_ext
        if ins.kind == "comment":
            write_line(ins.text)
            return
        if ins.kind == "prime":
            args: Dict[str, float] = {}
            emit_e(args, ins.de)
            f_prime = ins.f or UNRETRACT_FEED
            write_line(f"G1 E{format_float(args['E'])} F{format_float(f_prime)}\n")
            est.e_only(abs(ins.de), f_prime / 60.0)
            if ins.de > 0:
                final_ext += ins.de
                if count_added:
                    added_ext += ins.de
            return
        nx = ins.x if ins.x is not None else x
        ny = ins.y if ins.y is not None else y
        nz = ins.z if ins.z is not None else z
        out_x = nx - x if coord_relative and ins.x is not None else nx
        out_y = ny - y if coord_relative and ins.y is not None else ny
        out_z = nz - z if coord_relative and ins.z is not None else nz
        if ins.kind == "travel":
            parts = ["G0"]
            if ins.x is not None:
                parts.append(f"X{format_float(out_x)}")
            if ins.y is not None:
                parts.append(f"Y{format_float(out_y)}")
            if ins.z is not None:
                parts.append(f"Z{format_float(out_z)}")
            f_val = ins.f or travel_feed
            parts.append(f"F{format_float(f_val)}")
            write_line(" ".join(parts) + "\n")
            d = simulate(nx, ny, nz, f_val)
            if count_added:
                added_path += d
            return
        args = {}
        emit_e(args, ins.de)
        f_val = ins.f or feed
        if ins.z is not None:
            write_line(
                f"G1 X{format_float(out_x)} Y{format_float(out_y)} Z{format_float(out_z)}"
                f" E{format_float(args['E'])} F{format_float(f_val)}\n"
            )
        else:
            write_line(
                f"G1 X{format_float(out_x)} Y{format_float(out_y)}"
                f" E{format_float(args['E'])} F{format_float(f_val)}\n"
            )
        d = simulate(nx, ny, nz, f_val)
        if count_added:
            added_path += d
        if ins.de > 0:
            final_ext += ins.de
            if count_added:
                added_ext += ins.de

    for info in infos:
        line = info.line
        if info.kind == "m83":
            relative = True
            write_line(render_line(line) if line.command else line.raw)
        elif info.kind == "m82":
            relative = False
            write_line(render_line(line) if line.command else line.raw)
        elif info.kind == "g91":
            coord_relative = True
            write_line(render_line(line) if line.command else line.raw)
        elif info.kind == "g90":
            coord_relative = False
            write_line(render_line(line) if line.command else line.raw)
        elif info.kind == "g92":
            if "X" in line.args:
                x = line.args["X"]
            if "Y" in line.args:
                y = line.args["Y"]
            if "Z" in line.args:
                z = line.args["Z"]
            if "E" in line.args:
                e_cursor = line.args["E"]
            write_line(render_line(line))
        elif info.kind == "move":
            if "F" in line.args and line.args["F"] > 0:
                feed = line.args["F"]
            if info.index in replacements:
                for ins in replacements[info.index]:
                    emit_ins(ins, count_added=False)
            elif "E" in line.args:
                de = info.de * de_scale.get(info.index, 1.0)
                e_args: Dict[str, float] = {}
                emit_e(e_args, de)
                write_line(render_command_with_e(
                    line.command or "G1",
                    line.args,
                    e_args["E"],
                    line.comment,
                ))
                if de > 0:
                    final_ext += de
                if simulate(info.x1, info.y1, info.z1, feed) <= 1e-9 and de != 0.0 and feed > 0:
                    est.e_only(abs(de), feed / 60.0)
            else:
                write_line(line.raw if line.raw.endswith("\n") else line.raw + "\n")
                simulate(info.x1, info.y1, info.z1, feed)
        else:
            if line.command == "M204":
                est.set_accel(line.args.get("P", line.args.get("S", 0.0)))
            write_line(line.raw if line.raw.endswith("\n") else line.raw + "\n")
        key = info.index + 1
        if key in insertions:
            for block in insertions[key]:
                for ins in block:
                    emit_ins(ins)
            write_line(f"G1 F{format_float(feed)}\n")
    est.flush()
    return est.time, final_path, final_ext, added_path, added_ext


def layer_heights(
    loops: Sequence[Loop], prev_map: Optional[Dict[int, Loop]] = None
) -> Dict[int, float]:
    """每个回路的层高。

    优先用连通体配对结果（本回路 z − 配对的下层回路 z）——不同岛屿
    z 网格不对齐时全局层级差是错的，实际配对的 z 差才是该连通体的
    真实层高。未配对（首层/孤岛）回退到层级代表 z 之差；首层返回 0。"""
    reps, loop_level = cluster_z_levels(loops)
    heights: Dict[int, float] = {}
    for loop in loops:
        prev = prev_map.get(id(loop)) if prev_map else None
        if prev is not None:
            heights[id(loop)] = loop.z - prev.z
            continue
        li = loop_level[id(loop)]
        heights[id(loop)] = (reps[li] - reps[li - 1]) if li > 0 else 0.0
    return heights


class Progress:
    """控制台进度条（stderr、\r 原地刷新、100ms 节流）。

    Orca 调后处理脚本时捕获 stdout 解析结果，进度只能走 stderr；
    非终端（stderr 被重定向）时自动降级为每阶段一行。"""

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled and hasattr(sys.stderr, "write")
        self.is_tty = bool(getattr(sys.stderr, "isatty", lambda: False)())
        self._last = 0.0
        self._stage = ""

    def stage(self, name: str) -> None:
        if not self.enabled:
            return
        if self.is_tty and self._stage:
            sys.stderr.write("\n")
        self._stage = name
        self._last = 0.0
        self.update(0, 1)

    def update(self, done: int, total: int) -> None:
        if not self.enabled:
            return
        now = time.monotonic()
        if done < total and now - self._last < 0.1:
            return
        self._last = now
        frac = done / total if total > 0 else 1.0
        if self.is_tty:
            width = 28
            filled = int(width * frac)
            bar = "#" * filled + "-" * (width - filled)
            sys.stderr.write(f"\rBOWP {self._stage:<8s} [{bar}] {frac*100:5.1f}%")
        elif done == 0 or done >= total:
            sys.stderr.write(f"BOWP {self._stage} {frac*100:.0f}%\n")
        sys.stderr.flush()

    def close(self) -> None:
        if self.enabled and self.is_tty:
            sys.stderr.write("\n")
            sys.stderr.flush()


def decide_mode(
    loop: Loop,
    config: Dict[str, Any],
    h: float,
    prev_loop: Optional[Loop],
    next_loop: Optional[Loop],
    sample_step: float,
) -> Tuple[str, str]:
    """回路最终处理模式的唯一判定入口。返回 (mode, reason)。

    mode: none / spiral / scarf / flat；reason 供审计与总结统计。
    run_processor 与外部审计共用此函数，保证"为什么没超分"可追溯。"""
    seam_mode = str(config.get("seam_mode", "spiral"))
    wall_limit = fnum(config, "wall_length_limit_mm")
    total_len = loop.total_length
    if total_len < wall_limit:
        return "none", "below_wall_limit"
    mode = seam_mode
    reason = "requested"
    if mode == "spiral":
        if h <= 0.02:
            mode, reason = "scarf", "no_layer_height"
        elif not loop_is_closed(loop):
            mode, reason = "scarf", "open_loop"
    if mode == "scarf" and next_loop is None:
        mode, reason = "flat", reason + "+no_next_layer"
    max_overhang = fnum(config, "max_overhang_deg")
    if mode in ("spiral", "scarf") and -89.9 < max_overhang < 0:
        interp_on = mode == "spiral" and bool(config.get("spiral_xy_interp_enabled", True))
        bad_arc = loop_overhang_arc_mm(
            loop, prev_loop, h, sample_step * 4, max_overhang,
            offset_scale=0.5 if interp_on else 1.0,
        )
        if bad_arc > max(1.0, 0.02 * total_len):
            return "flat", "overhang"
    return mode, reason


def run_processor(gcode_path: str, config: Dict[str, Any]) -> Tuple[int, EstimateStats]:
    progress = Progress(enabled=bool(config.get("progress_enabled", True)))
    progress.stage("read")
    total_bytes = max(1, os.path.getsize(gcode_path))
    bytes_read = 0
    lines: List[GCodeLine] = []
    seen_head = 0
    processed = False
    has_native_ironing = False
    native_ironing_layers: set[int] = set()
    scan_layer = -1
    with open(gcode_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            bytes_read += len(raw)
            if seen_head < 20:
                seen_head += 1
                if BOWP_TAG in raw:
                    processed = True
            if (
                "BOWP secondary pass" in raw
                or "BOWP scarf overlap" in raw
                or "BOWP spiral" in raw
                or "BOWP script ironing" in raw
                or "BOWP step ironing" in raw
            ):
                processed = True
            raw_upper = raw.upper()
            marker = raw[1:].strip().upper() if raw.startswith(";") else ""
            if marker and any(token in marker for token in LAYER_MARKERS):
                scan_layer += 1
            if ("TYPE:" in raw_upper or "FEATURE:" in raw_upper) and "IRONING" in raw_upper:
                has_native_ironing = True
                native_ironing_layers.add(max(scan_layer, 0))
            lines.append(parse_line(raw))
            if (len(lines) & 0x3FFFF) == 0:
                progress.update(min(bytes_read, total_bytes), total_bytes)
    progress.update(total_bytes, total_bytes)
    if processed:
        progress.close()
        return 0, empty_stats()

    infos, base_time, base_path, base_ext = annotate(lines)
    sample_step = max(0.05, fnum(config, "sample_step_mm"))
    progress.stage("loops")
    include_overhang_starts = fnum(config, "max_overhang_deg") <= -89.9
    loops, arc_loops_skipped = build_loops(infos, sample_step, include_overhang_starts)
    single_wall_mask_enabled = bool(config.get("single_wall_mask_enabled", False))
    inner_index_by_layer: Dict[int, SegmentGrid] = {}
    if single_wall_mask_enabled:
        progress.stage("inner walls")
        inner_by_layer = build_inner_wall_segments(infos, sample_step)
        inner_index_by_layer = {
            layer: SegmentGrid(segments)
            for layer, segments in inner_by_layer.items()
            if segments
        }
    # 邻层配对（连通体感知）先行；层高用实际配对的 z 差
    prev_map, next_map = match_neighbor_loops(loops)
    heights = layer_heights(loops, prev_map)
    by_layer: Dict[int, List[Loop]] = {}
    for lp in loops:
        by_layer.setdefault(lp.layer_index, []).append(lp)
    loop_area: Dict[int, float] = {}
    loop_depth: Dict[int, int] = {}
    for layer_loops in by_layer.values():
        closed = [lp for lp in layer_loops if loop_is_closed(lp)]
        for lp in closed:
            loop_area[id(lp)] = abs(loop_signed_area(lp))
        for lp in closed:
            area = loop_area[id(lp)]
            cx, cy = loop_centroid(lp)
            depth = 0
            for other in closed:
                if other is lp or loop_area[id(other)] <= area:
                    continue
                if point_in_loop(cx, cy, other):
                    depth += 1
            loop_depth[id(lp)] = depth

    def compatible_neighbor(loop: Loop, other: Optional[Loop]) -> Optional[Loop]:
        if other is None:
            return None
        d0 = loop_depth.get(id(loop))
        d1 = loop_depth.get(id(other))
        if d0 is None or d1 is None or d0 != d1:
            return None
        return other

    de_scale: Dict[int, float] = {}
    replacements: Dict[int, List[Ins]] = {}
    insertions: Dict[int, List[List[Ins]]] = {}
    seam_count = 0
    overhang_demoted = 0
    ironing_trimmed = 0
    script_ironing_lines = 0
    script_ironing_active = bool(config.get("script_ironing_enabled", True))
    # 熨烫修剪：记录每个螺旋回路的实际到达场（上行/下行插值位移），
    # 配对完成后拼装每层覆盖带——环带上被真实螺旋覆盖处的原生熨烫
    # 才删（熨烫会把螺旋斜面重新压平）；螺旋没到达的环带中部与带外
    # （大平面内部、真顶面）的熨烫原样保留
    spiral_reach: Dict[int, Tuple[Loop, List[float], List[float]]] = {}
    spiral_zone_pairs: List[Tuple[Loop, Loop]] = []
    if config.get("seam_processing_enabled", True):
        min_wall = fnum(config, "min_wall_length_mm")
        wall_limit = fnum(config, "wall_length_limit_mm")
        fallback_rel = max(0.0, min(0.45, fnum(config, "short_wall_fallback_rel")))
        scarf = max(0.0, fnum(config, "scarf_length_mm"))
        detail_width = max(0.05, fnum(config, "path_width_mm"))
        base = fnum(config, "primary_flow_scale")
        secondary_enabled = bool(config.get("secondary_pass_enabled", True))
        seam_mode = str(config.get("seam_mode", "spiral"))
        flatten = bool(config.get("spiral_flatten_enabled", True))
        max_overhang = fnum(config, "max_overhang_deg")
        progress.stage("seams")
        n_loops = len(loops)
        for loop_i, loop in enumerate(loops):
            progress.update(loop_i, n_loops)
            if loop_depth.get(id(loop), 0) % 2 == 1:
                continue
            total_len = loop.total_length
            verts = loop_vertices_xy(loop) if loop_is_closed(loop) else []
            if verts:
                xs = [x for x, _ in verts]
                ys = [y for _, y in verts]
                span = max(max(xs) - min(xs), max(ys) - min(ys))
                detail_area = loop_area.get(id(loop), abs(loop_signed_area(loop)))
            else:
                span = 0.0
                detail_area = 0.0
            skinny_detail = (
                span <= max(8.0, 2.0 * scarf)
                and detail_area <= detail_width * total_len
            )
            single_wall_masked = (
                single_wall_mask_enabled
                and loop_lacks_inner_wall_support(
                    loop,
                    inner_index_by_layer.get(loop.layer_index),
                    detail_width,
                    sample_step,
                )
            )
            # 三档：< 极限周长 -> 不处理（只二次整形）；
            # 极限 ~ min(设定长度, 2*斜拼) -> 兜底：斜拼长度按周长百分比缩放；
            # 以上 -> 正常处理
            loop_cfg = config
            if total_len < wall_limit or skinny_detail or single_wall_masked:
                short_loop = True
            elif total_len < max(min_wall, 2 * scarf):
                short_loop = True
                loop_cfg = dict(config, scarf_length_mm=total_len * fallback_rel)
            else:
                short_loop = False
            h = heights.get(id(loop), 0.0)
            # 自适应速度：以该回路外墙的原始速度为基准，按倍率缩放
            original_feed = infos[loop.segments[0].line_index].feed
            wall_feed = original_feed * max(0.05, fnum(config, "spiral_speed_scale"))
            secondary_for_loop = secondary_enabled and not short_loop
            prev_loop = compatible_neighbor(loop, prev_map.get(id(loop)))
            next_loop = compatible_neighbor(loop, next_map.get(id(loop)))
            block: List[Ins] = []
            sub: Optional[Loop] = None
            sub_geometry: Optional[Loop] = None

            def get_subdivided(geometry: bool = False) -> Loop:
                nonlocal sub, sub_geometry
                source = closed_loop_geometry(loop) if geometry else loop
                if geometry:
                    if sub_geometry is None:
                        sub_geometry = subdivide_loop(
                            source, generated_sample_step(source, loop_cfg, sample_step)
                        )
                    return sub_geometry
                if sub is None:
                    sub = subdivide_loop(source, generated_sample_step(source, loop_cfg, sample_step))
                return sub

            mode, mode_reason = decide_mode(
                loop, loop_cfg, h, prev_loop, next_loop, sample_step
            )
            if mode_reason == "overhang":
                overhang_demoted += 1
            if not short_loop:
                if mode == "spiral":
                    sub_loop = get_subdivided(True)
                    for seg in loop.segments:
                        replacements[seg.line_index] = []
                    block.extend(build_spiral(sub_loop, loop_cfg, h, wall_feed, flatten,
                                              prev_loop, next_loop))
                elif mode == "scarf" and h > 0.02:
                    sub_loop = get_subdivided()
                    loop_repl, overlap = build_scarf(sub_loop, loop_cfg, h, wall_feed)
                    replacements.update(loop_repl)
                    replaced = set(loop_repl.keys())
                    for seg in loop.segments:
                        if seg.line_index not in replaced:
                            de_scale[seg.line_index] = base
                    block.extend(overlap)
                else:
                    # 平面模式：按细分段加权平均每条原始行的流量缩放
                    sub_loop = get_subdivided()
                    acc: Dict[int, float] = {}
                    tot: Dict[int, float] = {}
                    traveled = 0.0
                    for seg in sub_loop.segments:
                        mid = traveled + seg.length * 0.5
                        sc = segment_scale(mid, sub_loop.total_length, loop_cfg)
                        acc[seg.line_index] = acc.get(seg.line_index, 0.0) + seg.de * sc
                        tot[seg.line_index] = tot.get(seg.line_index, 0.0) + seg.de
                        traveled += seg.length
                    for li, de_sum in tot.items():
                        if de_sum > 1e-12:
                            de_scale[li] = acc[li] / de_sum
            # 螺旋+插值后顶面轮廓已向下一层偏移 0.5；二次整形与排压圈
            # 跟随同一轮廓，避免骑在原始轮廓上悬空或刮蹭
            purge_retract = fnum(loop_cfg, "purge_retract_mm")
            follow = get_subdivided(mode == "spiral") if (secondary_for_loop or (purge_retract > 0 and not short_loop)) else loop
            if mode == "spiral" and bool(loop_cfg.get("spiral_xy_interp_enabled", True)):
                sub_ref = get_subdivided(True)
                next_offsets = normal_offsets_toward(sub_ref, next_loop, h)
                follow = offset_loop_toward(
                    sub_ref, next_loop, 0.5,
                    clamp_mm=3.0 * max(h, 0.05), offsets=next_offsets,
                )
            if secondary_for_loop:
                block.extend(build_secondary_pass(follow, loop_cfg, short_loop,
                                                  wall_feed=original_feed,
                                                  continuous=(mode == "spiral")))
            if block:
                # 插在最后一条墙体挤出行之后、回抽/擦拭(;WIPE_START)之前：
                # 螺旋若落进 WIPE 区，Orca 预览会当擦拭移动隐藏，且被回抽状态污染
                insert_key = loop.segments[-1].line_index + 1
                retract = pending_retract(infos, loop, insert_key)
                # 终点排压圈：零流量走一圈、回抽摊到整圈逐步完成，
                # 停止点无静止回抽的余压淤积。原始静止回抽/擦拭清零
                # （排压圈接管），差额在圈尾一次补齐使 E 收支与原文件一致
                if purge_retract > 0 and loop_is_closed(loop) and not short_loop:
                    original_w = neutralize_trailing_retract(infos, insert_key, de_scale)
                    take = max(purge_retract, original_w)
                    block.extend(build_purge_lap(follow, take, wall_feed))
                    if take > original_w + 1e-9:
                        block.append(Ins(kind="prime", de=take - original_w, f=UNRETRACT_FEED))
                insertions.setdefault(insert_key, []).append(wrap_block(block, retract))
            if not short_loop and mode == "spiral" and (
                bool(config.get("ironing_trim_enabled", True)) or script_ironing_active
            ):
                # 记录螺旋实际到达场：环带（本层轮廓~下一层轮廓）上一点
                # 的熨烫可删，当且仅当它被本层螺旋的上行到达（follow，
                # 插值 0.5+钳制后）或下一层螺旋底圈的下行到达覆盖。
                # 不能用"到下一层的全距离"当带宽：插值在轮廓失配处
                # （伪对应/钳制）会清零，那里的螺旋贴原轮廓走，环带
                # 中部没有任何覆盖，熨烫必须保留，否则露出内墙。
                # up/down 均为逐段实际值；下行场在配对完成后拼装
                nxt_loop = next_loop
                if bool(loop_cfg.get("spiral_xy_interp_enabled", True)):
                    sub_ref = get_subdivided(True)
                    up_covs = [
                        math.hypot(b.x1 - a.x1, b.y1 - a.y1)
                        for a, b in zip(sub_ref.segments, follow.segments)
                    ]
                    prev_offsets = normal_offsets_toward(sub_ref, prev_loop, h)
                    down_follow = offset_loop_toward(
                        sub_ref, prev_loop, 0.5,
                        clamp_mm=3.0 * max(h, 0.05), offsets=prev_offsets,
                    )
                    down_covs = [
                        math.hypot(b.x1 - a.x1, b.y1 - a.y1)
                        for a, b in zip(sub_ref.segments, down_follow.segments)
                    ]
                    spiral_reach[id(loop)] = (sub_ref, up_covs, down_covs)
                    if nxt_loop is not None:
                        spiral_zone_pairs.append((loop, nxt_loop))
            seam_count += 1
        progress.update(n_loops, n_loops)

    if script_ironing_active:
        progress.stage("script ironing")
        by_layer: Dict[int, List[Loop]] = {}
        for lp in loops:
            by_layer.setdefault(lp.layer_index, []).append(lp)
        z_levels, loop_z_level = cluster_z_levels(loops)
        by_z_level: Dict[int, List[Loop]] = {}
        for lp in loops:
            by_z_level.setdefault(loop_z_level[id(lp)], []).append(lp)
        closed_by_z_level: Dict[int, List[Loop]] = {
            level: [lp for lp in level_loops if loop_is_closed(lp)]
            for level, level_loops in by_z_level.items()
        }
        material_indexes: Dict[int, LayeredMaterialIndex] = {}
        active_z_level = -1
        loop_area: Dict[int, float] = {}
        loop_depth: Dict[int, int] = {}
        loop_bbox: Dict[int, Tuple[float, float, float, float]] = {}
        for layer_loops in by_layer.values():
            closed = [lp for lp in layer_loops if loop_is_closed(lp)]
            for lp in closed:
                loop_area[id(lp)] = abs(loop_signed_area(lp))
                geo = closed_loop_geometry(lp)
                xs = [seg.x1 for seg in geo.segments]
                ys = [seg.y1 for seg in geo.segments]
                loop_bbox[id(lp)] = (min(xs), min(ys), max(xs), max(ys))
            for lp in closed:
                area = loop_area[id(lp)]
                cx, cy = loop_centroid(lp)
                depth = 0
                for other in closed:
                    if other is lp or loop_area[id(other)] <= area:
                        continue
                    if point_in_loop(cx, cy, other):
                        depth += 1
                loop_depth[id(lp)] = depth
        n_loops = len(loops)
        for loop_i, loop in enumerate(loops):
            progress.update(loop_i, n_loops)
            if not loop_is_closed(loop) or loop.total_length < fnum(config, "wall_length_limit_mm"):
                continue
            if loop.layer_index in native_ironing_layers:
                continue
            if loop_depth.get(id(loop), 0) % 2 == 1:
                continue
            # 同层内被当前外轮廓包含的小轮廓视作孔/岛边界，裸露顶面填线不得跨过去
            area = loop_area.get(id(loop), abs(loop_signed_area(loop)))
            blockers = []
            for other in by_layer.get(loop.layer_index, []):
                if other is loop or not loop_is_closed(other):
                    continue
                if loop_area.get(id(other), abs(loop_signed_area(other))) >= area:
                    continue
                cx, cy = loop_centroid(other)
                if point_in_loop(cx, cy, loop):
                    blockers.append(other)
            mapped_next = compatible_neighbor(loop, next_map.get(id(loop)))
            z_level = loop_z_level[id(loop)]
            if z_level != active_z_level:
                for old_level in [level for level in material_indexes if level < z_level]:
                    del material_indexes[old_level]
                active_z_level = z_level
            next_z_level = z_level + 1
            next_z_loops: Sequence[Loop] = ()
            if (
                next_z_level < len(z_levels)
                and z_levels[next_z_level] - loop.z <= MAX_NEIGHBOR_DZ
            ):
                next_z_loops = closed_by_z_level.get(next_z_level, ())
            next_cover_loops = list(next_z_loops)
            current_material_loops = closed_by_z_level.get(z_level, [])
            current_material_index = material_indexes.get(z_level)
            if current_material_index is None:
                current_material_index = LayeredMaterialIndex(current_material_loops)
                material_indexes[z_level] = current_material_index
            next_cover_index: Optional[LayeredMaterialIndex] = None
            if next_cover_loops:
                next_cover_index = material_indexes.get(next_z_level)
                if next_cover_index is None:
                    next_cover_index = LayeredMaterialIndex(next_cover_loops)
                    material_indexes[next_z_level] = next_cover_index
            iron = build_script_ironing(
                loop,
                config,
                heights.get(id(loop), 0.0),
                mapped_next,
                spiral_reach.get(id(loop)),
                blockers,
                next_cover_loops,
                current_material_loops,
                next_cover_index,
                current_material_index,
            )
            if not iron:
                continue
            insert_key = loop.segments[-1].line_index + 1
            retract = pending_retract(infos, loop, insert_key)
            insertions.setdefault(insert_key, []).append(wrap_block(iron, retract))
            script_ironing_lines += sum(1 for ins in iron if ins.kind == "extrude" and ins.de > 0)
        progress.update(n_loops, n_loops)

    spiral_zones: Dict[int, List[Tuple[Loop, List[float]]]] = {}
    if spiral_zone_pairs:
        # 拼装每层覆盖带：每对 (本层, 下一层) 注册两条带——
        #   本层轮廓 + 上行到达场（本层螺旋向下一层插值的实际位移）
        #   下一层轮廓 + 下行到达场（下一层底圈向本层插值的实际位移）
        # 熨烫点贴近哪条轮廓、且在其实际到达半径内才算覆盖；环带
        # 中部两侧都够不到的区域不删（插值被伪对应/钳制清零处，
        # 螺旋贴原轮廓走，删了熨烫就露内墙）
        for loop, nxt_loop in spiral_zone_pairs:
            entry = spiral_reach.get(id(loop))
            if entry is None:
                continue
            sub_ref, up_covs, _ = entry
            zkey = round(loop.z, 2)
            if max(up_covs, default=0.0) > 0.05:
                spiral_zones.setdefault(zkey, []).append((sub_ref, up_covs))
            nxt_entry = spiral_reach.get(id(nxt_loop))
            if nxt_entry is not None:
                nsub, _, ndown = nxt_entry
                if max(ndown, default=0.0) > 0.05:
                    spiral_zones.setdefault(zkey, []).append((nsub, ndown))

    if spiral_zones:
        # 熨烫修剪：整条删除主要落在螺旋覆盖带内的原生熨烫笔画
        # （熨烫会把螺旋斜面重新压平；喷嘴热量本身就足以损伤斜面，
        # 因此连运动一起删除，不保留空走）。按"笔画"为单位裁决
        # （相邻挤出行 + 引导空移），避免删除个别行造成起点错位的
        # 飞线挤出；带内弧长占比 >50% 才删，plateau 内部笔画保留
        zone_zs = sorted(spiral_zones)

        def zones_for(z_val: float):
            zs = spiral_zones.get(round(z_val, 2))
            if zs is not None:
                return zs
            for zz in zone_zs:
                if abs(zz - z_val) <= 0.06:
                    return spiral_zones[zz]
            return None

        def in_zone(px: float, py: float, zones) -> bool:
            # 局部判据：找最近轮廓段，用该段的局部带宽 + 半线宽富余
            # 作为半径。垂直墙段带宽~0，贴着它的顶面熨烫不会误删
            for zloop, covs in zones:
                segs = zloop.segments
                best_d2 = float("inf")
                best_i = 0
                for i, seg in enumerate(segs):
                    ax, ay = seg.x0, seg.y0
                    bx, by = seg.x1, seg.y1
                    abx, aby = bx - ax, by - ay
                    ab2 = abx * abx + aby * aby
                    t = 0.0 if ab2 <= 1e-12 else max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
                    dx, dy = ax + abx * t - px, ay + aby * t - py
                    d2 = dx * dx + dy * dy
                    if d2 < best_d2:
                        best_d2 = d2
                        best_i = i
                # 相邻段取最大覆盖宽（宽度场沿弧长缓变，防边界骤降）
                n = len(covs)
                local = max(covs[(best_i - 1) % n], covs[best_i], covs[(best_i + 1) % n])
                # 显著性下限 0.1mm：更窄意味着近垂直墙，没有可损伤的
                # 斜面，贴边熨烫保留。带内 = 距离不超过局部带宽 + 半线
                # 宽富余（熨烫线中心可离带缘半线宽仍压到斜面）
                if local >= 0.1 and math.sqrt(best_d2) <= local + 0.25:
                    return True
            return False

        section = ""
        # 括号级删除。Orca 的回填(prime)在一组笔画之前、回抽+擦拭在
        # 之后，一对回填/回抽可包住多条笔画甚至跨 TYPE 边界。逐笔画
        # 配对必然产生孤儿 E 行：只删回填留回抽 → E 负漂移；原地保留
        # 成对纯 E 行 → 回填+回抽压力循环，每处渗出小凸点（预览成串
        # 白珠）。以完整括号为单元：
        #   [回填序列] (空移+笔画)... [回抽序列(含擦拭)]
        # 括号内笔画全部为待删熨烫 → 回填/回抽/擦拭/空移一起删（E 收支
        # 为零、无驻点脉冲）；部分待删 → 只删笔画+引导空移，E 行保留
        # （仍服务剩余笔画）。
        primes: List[LineInfo] = []
        retracts: List[LineInfo] = []
        travels: List[LineInfo] = []
        strokes: List[Tuple[List[LineInfo], List[LineInfo], bool]] = []  # (lead, run, is_ironing)
        lead: List[LineInfo] = []
        closing = False     # 已进入回抽序列

        def stroke_deletable(run: List[LineInfo]) -> bool:
            zones = zones_for(run[0].z1)
            if not zones:
                return False
            total_l = in_l = 0.0
            for info in run:
                L = math.hypot(info.x1 - info.x0, info.y1 - info.y0)
                total_l += L
                if in_zone((info.x0 + info.x1) / 2, (info.y0 + info.y1) / 2, zones):
                    in_l += L
            return total_l > 1e-9 and in_l / total_l > 0.5

        def close_bracket() -> None:
            nonlocal ironing_trimmed, primes, retracts, travels, strokes, lead
            nonlocal closing
            doomed = [s for s in strokes if s[2] and stroke_deletable(s[1])]
            if doomed:
                # 整删一致性与括号外状态无关：设进入时物理回抽深度 R，
                # 原括号后深度 = R - P - T（P=回填和>0，T=回抽和<0）。
                # 删除全部行后深度仍为 R，补一行 A = P + T 的纯 E 调整
                # 即恢复一致。A 通常是重启补偿差（如 +0.3），此时喷嘴
                # 处于深回抽态，静止 E 调整不出料、无凸点。A 过大说明
                # 括号结构异常，退回逐笔画删除（E 行保留）。
                adj = sum(i.de for i in primes) + sum(i.de for i in retracts)
                if (
                    primes
                    and retracts
                    and len(doomed) == len(strokes)
                    and adj <= 0.5
                ):
                    for info in primes + retracts + travels:
                        replacements.setdefault(info.index, [])
                    if abs(adj) > 1e-6:
                        replacements[primes[0].index] = [
                            Ins(kind="prime", de=adj, f=UNRETRACT_FEED)
                        ]
                    for ld, run, _ in strokes:
                        for info in ld + run:
                            replacements.setdefault(info.index, [])
                        ironing_trimmed += len(run)
                else:
                    for ld, run, _ in doomed:
                        for info in ld + run:
                            replacements.setdefault(info.index, [])
                        ironing_trimmed += len(run)
            primes = []
            retracts = []
            travels = []
            strokes = []
            lead = []
            closing = False

        for info in infos:
            c = info.line.comment
            if c.startswith(";") and "TYPE:" in c.upper():
                # 括号可跨 TYPE 边界（回填在标记之前），只更新分区
                section = "ironing" if "IRONING" in c.upper() else "other"
                continue
            if info.kind == "g92":
                # E 重置对括号透明（渲染层按增量重排 E 值）；配对的
                # 回抽常在换层 G92 之后，在此断开会留下孤儿回填
                continue
            if info.kind != "move":
                continue
            has_xy = "X" in info.line.args or "Y" in info.line.args
            if "Z" in info.line.args:
                # 带 Z 的移动（层间空移/抬升）是括号硬边界，始终保留
                close_bracket()
                continue
            if info.de < -1e-9:
                # 回抽（纯 E 或带运动的擦拭）：进入收尾序列
                closing = True
                retracts.append(info)
                if lead:
                    travels.extend(lead)
                    lead = []
                continue
            if not has_xy and abs(info.de) <= 1e-9:
                # F-only 行：保留，不参与、不打断回抽序列
                # （Orca 在回抽与擦拭之间常插 G1 F2400）
                continue
            if closing:
                close_bracket()
            if info.de > 1e-9 and not has_xy:
                # 回填：开启新括号（连续多段回填并入同一括号）
                if strokes:
                    close_bracket()
                primes.append(info)
                if lead:
                    travels.extend(lead)
                    lead = []
            elif info.de > 1e-9:
                is_iron = section == "ironing"
                if not lead and strokes and strokes[-1][2] == is_iron:
                    strokes[-1][1].append(info)
                else:
                    strokes.append((lead, [info], is_iron))
                    lead = []
            elif has_xy:
                lead.append(info)
            # F-only 行：保留，不参与
        close_bracket()

        # 收敛：插入块（排压圈等）以回填收尾、其后原本隔着熨烫才到
        # 原生回抽——熨烫整删后回填与回抽在原地对撞（驻点脉冲）。
        # 把两者合并为一条净量 prime（回填-回抽），回抽行删除，
        # E 收支不变、无脉冲
        for key, blocks in insertions.items():
            last_prime: Optional[Ins] = None
            for blk in blocks:
                for ins in blk:
                    if ins.kind in ("extrude", "travel", "prime"):
                        last_prime = ins if (ins.kind == "prime" and ins.de > 0) else None
            if last_prime is None:
                continue
            j = key
            merged = False
            while j < len(infos) and not merged:
                nxt = infos[j]
                if nxt.kind in ("m82", "m83"):
                    break
                if nxt.kind != "move":
                    j += 1
                    continue
                if nxt.index in replacements and not replacements[nxt.index]:
                    j += 1
                    continue
                has_xyz = ("X" in nxt.line.args or "Y" in nxt.line.args
                           or "Z" in nxt.line.args)
                if has_xyz:
                    break
                if nxt.de < -1e-9:
                    # 有效回抽量须计入 de_scale（排压圈已把原回抽清零）
                    eff = nxt.de * de_scale.get(nxt.index, 1.0)
                    if eff < -1e-9:
                        last_prime.de += eff
                        replacements[nxt.index] = []
                        merged = True
                    else:
                        break
                elif abs(nxt.de) > 1e-9:
                    break
                j += 1

    progress.stage("render")
    temp_path = ""
    target_dir = os.path.dirname(os.path.abspath(gcode_path))

    def render_to_temp() -> Tuple[float, float, float, float, float]:
        nonlocal temp_path
        os.makedirs(target_dir, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            delete=False,
            # 必须与目标同目录（同盘）：os.replace 跨盘在 Windows 上失败
            dir=target_dir,
            prefix=".bowp-",
            suffix=".gcode",
        ) as temp_file:
            temp_path = temp_file.name
            temp_file.write("; BOWP processed by better_outer_wall_processing v0.3.0\n")
            return render_output_stream(
                infos, de_scale, replacements, insertions, config, temp_file.write
            )

    try:
        try:
            final_time, final_path, final_ext, added_path, added_ext = render_to_temp()
        except FileNotFoundError:
            if temp_path:
                try:
                    os.remove(temp_path)
                except OSError:
                    pass
            temp_path = ""
            final_time, final_path, final_ext, added_path, added_ext = render_to_temp()
        progress.stage("write")
        os.replace(temp_path, gcode_path)
        temp_path = ""
    finally:
        if temp_path:
            try:
                os.remove(temp_path)
            except OSError:
                pass
    progress.update(1, 1)
    progress.close()

    stats = EstimateStats(
        base_time_s=base_time,
        final_time_s=final_time,
        added_time_s=max(0.0, final_time - base_time),
        base_path_mm=base_path,
        final_path_mm=final_path,
        added_path_mm=added_path,
        base_extrusion_mm=base_ext,
        final_extrusion_mm=final_ext,
        added_extrusion_mm=added_ext,
        arc_loops_skipped=arc_loops_skipped,
        overhang_demoted=overhang_demoted,
        ironing_trimmed=ironing_trimmed,
        script_ironing_lines=script_ironing_lines,
    )
    return seam_count, stats


def format_percent(delta: float, base: float) -> str:
    if abs(base) < 1e-9:
        return "N/A"
    return f"{(delta / base) * 100:.2f}%"


def format_duration(seconds: float) -> str:
    seconds = max(0.0, seconds)
    minutes, sec = divmod(int(round(seconds)), 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours}小时{minutes}分{sec}秒"
    if minutes:
        return f"{minutes}分{sec}秒"
    return f"{sec}秒"


PROCESSED_DIR = os.path.join(PLUGIN_DIR, "processed")


def save_processed_copy(gcode_path: str) -> str:
    """把处理后的 G-code 存一份到插件目录下，供用户在 Orca 里打开预览。

    Orca 的后处理作用于导出临时文件，用户平时看不到结果；这份副本可以
    拖进 OrcaSlicer 窗口（或 文件 > 打开，选 .gcode）用完整 3D 预览查看
    螺旋路径、流量着色。只保留最近 5 份。"""
    import time as _time

    os.makedirs(PROCESSED_DIR, exist_ok=True)
    stem = os.path.splitext(os.path.basename(gcode_path))[0]
    stamp = _time.strftime("%Y%m%d-%H%M%S")
    dest = os.path.join(PROCESSED_DIR, f"{stem}-{stamp}.gcode")
    import shutil as _shutil

    _shutil.copyfile(gcode_path, dest)
    copies = sorted(
        (os.path.join(PROCESSED_DIR, n) for n in os.listdir(PROCESSED_DIR) if n.endswith(".gcode")),
        key=os.path.getmtime,
    )
    for old in copies[:-5]:
        try:
            os.remove(old)
        except OSError:
            pass
    return dest


def show_summary_dialog(summary: str, copy_path: str = "") -> None:
    """在独立进程中弹出总结窗口，不阻塞后处理流程（Orca 同步等待脚本退出）。

    有处理副本时提供"查看处理后G-code"按钮：打开资源管理器选中该文件，
    拖入 OrcaSlicer 即可 3D 预览。"""
    import subprocess
    import sys

    child = (
        "import sys, subprocess, tkinter as tk\n"
        "data = sys.stdin.buffer.read().decode('utf-8')\n"
        "text, _, copy_path = data.partition('\\x00')\n"
        "root = tk.Tk()\n"
        "root.title('更好的外墙处理')\n"
        "root.attributes('-topmost', True)\n"
        "tk.Label(root, text=text, justify='left', padx=16, pady=12).pack()\n"
        "bar = tk.Frame(root); bar.pack(pady=(0, 12))\n"
        "if copy_path:\n"
        "    def reveal():\n"
        "        subprocess.Popen(['explorer', '/select,', copy_path])\n"
        "    tk.Button(bar, text='查看处理后G-code', command=reveal).pack(side='left', padx=6)\n"
        "tk.Button(bar, text='确定', command=root.destroy).pack(side='left', padx=6)\n"
        "root.after(60000, root.destroy)\n"
        "root.mainloop()\n"
    )
    try:
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        proc = subprocess.Popen(
            [sys.executable, "-c", child],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=creationflags,
        )
        proc.stdin.write((summary + "\x00" + copy_path).encode("utf-8"))
        proc.stdin.close()
    except Exception as exc:
        debug_log(f"show_summary_dialog failed: {exc}")


def build_html(config: Dict[str, Any]) -> str:
    json_config = json.dumps(config, ensure_ascii=False)
    rows: List[str] = []
    for field in CONFIG_FIELDS:
        key = field["key"]
        label = field["label"]
        kind = field["kind"]
        if kind == "bool":
            control = f'<input id="{key}" type="checkbox">'
        elif kind == "enum":
            options = "".join(f'<option value="{value}">{value}</option>' for value in field["options"])
            control = f'<select id="{key}">{options}</select>'
        elif kind == "text":
            control = f'<input id="{key}" type="text">'
        else:
            step = "1" if kind == "int" else "0.01"
            control = f'<input id="{key}" type="number" step="{step}">'
        rows.append(f"  <label>{label}</label>{control}")
    return f"""
<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<title>更好的外墙处理</title>
<style>
body {{ font: 14px/1.5 var(--orca-font); margin: 18px; }}
h1, h2 {{ margin: 0 0 12px; }}
.grid {{ display: grid; grid-template-columns: 1fr 140px; gap: 8px 12px; align-items: center; }}
label {{ color: var(--orca-muted); }}
input, select {{ width: 100%; box-sizing: border-box; }}
textarea {{ width: 100%; min-height: 260px; box-sizing: border-box; white-space: pre-wrap; }}
.actions {{ display: flex; gap: 10px; margin-top: 14px; }}
</style>
<body>
<h1>更好的外墙处理</h1>
<p>保存后，后处理能力会在下次导出 G-code 时使用这些参数。</p>
<div class="grid">
{chr(10).join(rows)}
</div>
<div class="actions">
  <button onclick="save()">保存</button>
  <button onclick="orca.close()">取消</button>
</div>
<h2>中文说明</h2>
<textarea readonly>{MANUAL_TEXT}</textarea>
<script>
const cfg = {json_config};
for (const [k, v] of Object.entries(cfg)) {{
  const el = document.getElementById(k);
  if (!el) continue;
  if (el.type === "checkbox") el.checked = !!v;
  else el.value = v;
}}
function readValue(el) {{
  if (el.type === "checkbox") return el.checked;
  if (el.type === "number") return Number(el.value);
  return el.value;
}}
function save() {{
  const out = {{}};
  for (const id of Object.keys(cfg)) {{
    const el = document.getElementById(id);
    if (el) out[id] = readValue(el);
  }}
  orca.submit(out);
}}
</script>
</body>
</html>
"""


class BetterOuterWallProcessing(orca.gcode.GCodePluginCapabilityBase):
    def get_name(self):
        return "更好的外墙处理"

    def execute(self, ctx):
        config = load_config()
        if not config.get("enabled", True):
            return orca.ExecutionResult.skipped("插件已关闭")
        try:
            seam_count, stats = run_processor(ctx.gcode_path, config)
            message = (
                f"已处理 {seam_count} 个外墙回路。"
                f" 预计原始耗时 {stats.base_time_s/60:.1f} 分钟，插件后 {stats.final_time_s/60:.1f} 分钟，"
                f" 增加 {stats.added_time_s/60:.1f} 分钟。"
            )
            if stats.script_ironing_lines:
                message += f" 脚本熨烫生成 {stats.script_ironing_lines} 条线段。"
            if stats.arc_loops_skipped:
                message += f" 无法展开的圆弧外墙回路已跳过 {stats.arc_loops_skipped} 个。"
            return orca.ExecutionResult.success(message)
        except Exception as exc:
            return orca.ExecutionResult.failure(
                orca.PluginResult.RecoverableError,
                f"更好的外墙处理失败: {exc}",
            )


class BetterOuterWallSettings(orca.script.ScriptPluginCapabilityBase):
    def get_name(self):
        return "更好的外墙处理设置"

    def execute(self):
        config = load_config()
        result = orca.host.ui.show_dialog(
            html=build_html(config),
            title="更好的外墙处理",
            width=920,
            height=860,
        )
        if result is None:
            return orca.ExecutionResult.skipped("已取消")
        try:
            merged = dict(DEFAULT_CONFIG)
            merged.update(result)
            save_config(merged)
            return orca.ExecutionResult.success("配置已保存")
        except Exception as exc:
            return orca.ExecutionResult.failure(
                orca.PluginResult.RecoverableError,
                f"保存配置失败: {exc}",
            )


@orca.plugin
class BetterOuterWallPlugin(orca.base):
    def register_capabilities(self):
        orca.register_capability(BetterOuterWallProcessing)
        orca.register_capability(BetterOuterWallSettings)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        raise SystemExit(
            "usage: better_outer_wall_processing.py [options] <gcode-path>\n"
            "example: better_outer_wall_processing.py --seam-superres 8 --secondary-speed-scale 1.0 file.gcode"
        )
    cfg, gcode_path = apply_cli_overrides(load_config(), sys.argv[1:])
    seams, stats = run_processor(gcode_path, cfg)
    summary = (
        f"更好的外墙处理已完成\n\n"
        f"处理的外墙回路: {seams}\n"
        f"跳过的无法展开圆弧回路: {stats.arc_loops_skipped}\n"
        f"悬垂降级回路(flat): {stats.overhang_demoted}\n"
        f"修剪的熨烫行(覆盖区): {stats.ironing_trimmed}\n"
        f"脚本熨烫线段: {stats.script_ironing_lines}\n"
        f"预计原始耗时: {format_duration(stats.base_time_s)}\n"
        f"预计插件后耗时: {format_duration(stats.final_time_s)}\n"
        f"增加耗时: {format_duration(stats.added_time_s)} ({format_percent(stats.added_time_s, stats.base_time_s)})\n"
        f"原始路径长度: {stats.base_path_mm:.2f} mm\n"
        f"插件后路径长度: {stats.final_path_mm:.2f} mm\n"
        f"增加路径长度: {stats.added_path_mm:.2f} mm ({format_percent(stats.added_path_mm, stats.base_path_mm)})\n"
        f"原始挤出量: {stats.base_extrusion_mm:.3f} mm\n"
        f"插件后挤出量: {stats.final_extrusion_mm:.3f} mm\n"
        f"增加挤出量: {stats.added_extrusion_mm:.3f} mm ({format_percent(stats.added_extrusion_mm, stats.base_extrusion_mm)})\n"
    )
    copy_path = ""
    if seams > 0 and cfg.get("keep_processed_copy", True):
        try:
            copy_path = save_processed_copy(gcode_path)
            summary += f"\n处理副本（可拖入 Orca 预览）:\n{copy_path}\n"
        except Exception as exc:
            debug_log(f"save_processed_copy failed: {exc}")
    show_summary_dialog(summary, copy_path)
    print(
        "processed seam loops="
        f"{seams}, arc loops skipped={stats.arc_loops_skipped}, "
        f"base_time_s={stats.base_time_s:.3f}, "
        f"final_time_s={stats.final_time_s:.3f}, "
        f"added_time_s={stats.added_time_s:.3f}, "
        f"base_path_mm={stats.base_path_mm:.3f}, "
        f"final_path_mm={stats.final_path_mm:.3f}, "
        f"added_path_mm={stats.added_path_mm:.3f}, "
        f"base_extrusion_mm={stats.base_extrusion_mm:.3f}, "
        f"final_extrusion_mm={stats.final_extrusion_mm:.3f}, "
        f"added_extrusion_mm={stats.added_extrusion_mm:.3f}"
    )
