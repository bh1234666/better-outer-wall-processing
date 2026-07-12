from __future__ import annotations

import argparse
import bisect
import math
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from typing import DefaultDict, Iterable


CELL = 2.0
SCANLINE_CELL_MM = 2.0
TRAVEL_GAP_MM = 0.5
NEIGHBOR_DZ = 0.62
FLOW_DENSITY_FLOOR = 0.0015
ARC_STEP_MM = 0.5
FLAT_LOOP_Z_TOL = 0.03


@dataclass(slots=True)
class Seg:
    x0: float
    y0: float
    x1: float
    y1: float
    z: float
    de: float
    feed: float
    kind: str
    generated: bool = False


@dataclass(slots=True)
class Loop:
    layer: int
    z: float
    segs: list[Seg]
    area: float = 0.0
    y_min: float = field(init=False)
    y_max: float = field(init=False)
    z_min: float = field(init=False)
    z_max: float = field(init=False)
    scanline_grid: dict[int, list[int]] | None = field(default=None, repr=False)
    interval_cache: dict[float, list[tuple[float, float]]] = field(default_factory=dict, repr=False)

    def __post_init__(self) -> None:
        ys = [v for seg in self.segs for v in (seg.y0, seg.y1)]
        self.y_min = min(ys) if ys else 0.0
        self.y_max = max(ys) if ys else 0.0
        zs = [seg.z for seg in self.segs]
        self.z_min = min(zs) if zs else self.z
        self.z_max = max(zs) if zs else self.z

    @property
    def z_span(self) -> float:
        return self.z_max - self.z_min


@dataclass(slots=True)
class Continuity:
    max_feed_ratio: float = 1.0
    max_flow_ratio: float = 1.0
    max_transition_feed_ratio: float = 1.0
    max_transition_flow_ratio: float = 1.0
    feed_at: tuple[float, float, float] | None = None
    flow_at: tuple[float, float, float] | None = None
    transition_feed_at: tuple[float, float, float] | None = None
    transition_flow_at: tuple[float, float, float] | None = None
    pairs: int = 0
    transition_pairs: int = 0


@dataclass(slots=True)
class Parsed:
    loops: DefaultDict[int, list[Loop]] = field(default_factory=lambda: defaultdict(list))
    wall: DefaultDict[int, list[Seg]] = field(default_factory=lambda: defaultdict(list))
    cover: DefaultDict[int, list[Seg]] = field(default_factory=lambda: defaultdict(list))
    finish: DefaultDict[int, list[Seg]] = field(default_factory=lambda: defaultdict(list))
    continuity: Continuity = field(default_factory=Continuity)


@dataclass(slots=True)
class PassStats:
    min_count: int = 0
    max_count: int = 0
    avg: float = 0.0
    stdev: float = 0.0
    samples: int = 0

    @property
    def cv(self) -> float:
        return self.stdev / self.avg if self.avg > 1e-9 else 0.0


class SegIndex:
    def __init__(self, segs: Iterable[Seg]) -> None:
        self.segs = list(segs)
        self.grid: dict[tuple[int, int], list[int]] = defaultdict(list)
        inv = 1.0 / CELL
        for i, seg in enumerate(self.segs):
            for gx in range(int(min(seg.x0, seg.x1) * inv), int(max(seg.x0, seg.x1) * inv) + 1):
                for gy in range(int(min(seg.y0, seg.y1) * inv), int(max(seg.y0, seg.y1) * inv) + 1):
                    self.grid[(gx, gy)].append(i)

    def nearest(self, px: float, py: float, radius: float) -> float:
        if not self.segs:
            return float("inf")
        inv = 1.0 / CELL
        cgx, cgy = int(px * inv), int(py * inv)
        rings = int(radius * inv) + 2
        best = float("inf")
        seen: set[int] = set()
        for ring in range(rings + 1):
            if best <= (ring - 1) * CELL and ring >= 2:
                break
            for cell in ring_cells(cgx, cgy, ring):
                for i in self.grid.get(cell, ()):
                    if i in seen:
                        continue
                    seen.add(i)
                    d = seg_distance(px, py, self.segs[i])
                    if d < best:
                        best = d
        return best

    def count_within(self, px: float, py: float, radius: float) -> int:
        if not self.segs:
            return 0
        inv = 1.0 / CELL
        cgx, cgy = int(px * inv), int(py * inv)
        rings = int(radius * inv) + 2
        count = 0
        seen: set[int] = set()
        for ring in range(rings + 1):
            for cell in ring_cells(cgx, cgy, ring):
                for i in self.grid.get(cell, ()):
                    if i in seen:
                        continue
                    seen.add(i)
                    if seg_distance(px, py, self.segs[i]) <= radius:
                        count += 1
        return count

    def pass_count_within(self, px: float, py: float, radius: float) -> int:
        if not self.segs:
            return 0
        inv = 1.0 / CELL
        cgx, cgy = int(px * inv), int(py * inv)
        rings = int(radius * inv) + 2
        hits: list[int] = []
        seen: set[int] = set()
        for ring in range(rings + 1):
            for cell in ring_cells(cgx, cgy, ring):
                for i in self.grid.get(cell, ()):
                    if i in seen:
                        continue
                    seen.add(i)
                    if seg_distance(px, py, self.segs[i]) <= radius:
                        hits.append(i)
        if not hits:
            return 0
        hits.sort()
        count = 1
        prev_i = hits[0]
        prev_seg = self.segs[prev_i]
        for i in hits[1:]:
            seg = self.segs[i]
            continuous = (
                i == prev_i + 1
                and seg.kind == prev_seg.kind
                and abs(seg.z - prev_seg.z) <= 0.02
                and math.hypot(seg.x0 - prev_seg.x1, seg.y0 - prev_seg.y1) <= TRAVEL_GAP_MM
            )
            if not continuous:
                count += 1
            prev_i = i
            prev_seg = seg
        return count


def ring_cells(cgx: int, cgy: int, ring: int) -> list[tuple[int, int]]:
    if ring == 0:
        return [(cgx, cgy)]
    cells = [(cgx + k, cgy - ring) for k in range(-ring, ring + 1)]
    cells += [(cgx + k, cgy + ring) for k in range(-ring, ring + 1)]
    cells += [(cgx - ring, cgy + k) for k in range(-ring + 1, ring)]
    cells += [(cgx + ring, cgy + k) for k in range(-ring + 1, ring)]
    return cells


def seg_distance(px: float, py: float, seg: Seg) -> float:
    dx, dy = seg.x1 - seg.x0, seg.y1 - seg.y0
    d2 = dx * dx + dy * dy
    t = 0.0 if d2 <= 1e-12 else max(0.0, min(1.0, ((px - seg.x0) * dx + (py - seg.y0) * dy) / d2))
    return math.hypot(seg.x0 + dx * t - px, seg.y0 + dy * t - py)


def loop_area(segs: list[Seg]) -> float:
    return 0.5 * sum(seg.x0 * seg.y1 - seg.x1 * seg.y0 for seg in segs)


def loop_closed(segs: list[Seg]) -> bool:
    if len(segs) < 3:
        return False
    return math.hypot(segs[0].x0 - segs[-1].x1, segs[0].y0 - segs[-1].y1) <= 0.25


def parse_tokens(line: str) -> dict[str, float]:
    args: dict[str, float] = {}
    for tok in line.split(";")[0].split()[1:]:
        if len(tok) < 2:
            continue
        try:
            args[tok[0].upper()] = float(tok[1:])
        except ValueError:
            pass
    return args


def expand_arc_segments(
    cmd: str,
    args: dict[str, float],
    x: float,
    y: float,
    z: float,
    nx: float,
    ny: float,
    nz: float,
    de: float,
    feed: float,
    kind: str,
    generated: bool = False,
) -> list[Seg] | None:
    if cmd not in ("G2", "G3") or ("I" not in args and "J" not in args):
        return None
    cx, cy = x + args.get("I", 0.0), y + args.get("J", 0.0)
    r0 = math.hypot(x - cx, y - cy)
    r1 = math.hypot(nx - cx, ny - cy)
    if r0 <= 1e-6 or abs(r0 - r1) > max(0.05, 0.01 * r0):
        return None
    a0 = math.atan2(y - cy, x - cx)
    a1 = math.atan2(ny - cy, nx - cx)
    sweep = a1 - a0
    if cmd == "G2":
        while sweep >= -1e-9:
            sweep -= 2 * math.pi
    else:
        while sweep <= 1e-9:
            sweep += 2 * math.pi
    arc_len = abs(sweep) * r0
    if arc_len <= 1e-6:
        return None
    n = max(2, math.ceil(arc_len / ARC_STEP_MM))
    out: list[Seg] = []
    px, py, pz = x, y, z
    for i in range(1, n + 1):
        t = i / n
        ang = a0 + sweep * t
        qx = cx + r0 * math.cos(ang)
        qy = cy + r0 * math.sin(ang)
        qz = z + (nz - z) * t
        out.append(Seg(px, py, qx, qy, qz, de / n, feed, kind, generated))
        px, py, pz = qx, qy, qz
    return out


def parse_file(path: str, include_overhang_starts: bool = False) -> Parsed:
    parsed = Parsed()
    x = y = z = e = 0.0
    feed = 1800.0
    rel = False
    coord_rel = False
    layer = -1
    section = ""
    bowp = ""
    current: list[Seg] = []
    last_gen: Seg | None = None
    last_transition_gen: Seg | None = None

    def flush_loop() -> None:
        nonlocal current
        if loop_closed(current):
            lp = Loop(layer=layer, z=current[0].z, segs=current[:], area=loop_area(current))
            parsed.loops[layer].append(lp)
        current = []

    def update_continuity(seg: Seg) -> None:
        nonlocal last_gen, last_transition_gen
        if not seg.generated or seg.kind not in ("bowp", "iron"):
            last_gen = None
            last_transition_gen = None
            return
        length = math.hypot(seg.x1 - seg.x0, seg.y1 - seg.y0)
        density = seg.de / length if length > 1e-9 else 0.0

        def comparable(prev: Seg) -> tuple[float, float, float] | None:
            gap = math.hypot(seg.x0 - prev.x1, seg.y0 - prev.y1)
            if gap > 0.35:
                return None
            prev_len = math.hypot(prev.x1 - prev.x0, prev.y1 - prev.y0)
            prev_density = prev.de / prev_len if prev_len > 1e-9 else 0.0
            return gap, prev_len, prev_density

        def flow_ratio(prev: Seg, prev_len: float, prev_density: float) -> float | None:
            if (
                density > 1e-6
                and prev_density > 1e-6
                and density >= FLOW_DENSITY_FLOOR
                and prev_density >= FLOW_DENSITY_FLOOR
                and seg.de >= 0.0002
                and prev.de >= 0.0002
                and length >= 0.02
                and prev_len >= 0.02
            ):
                return max(density, prev_density) / min(density, prev_density)
            return None

        if last_gen is not None and last_gen.kind == seg.kind:
            values = comparable(last_gen)
            if values is not None:
                _, last_len, last_density = values
                if seg.feed > 0 and last_gen.feed > 0:
                    ratio = max(seg.feed, last_gen.feed) / max(1e-9, min(seg.feed, last_gen.feed))
                    if ratio > parsed.continuity.max_feed_ratio:
                        parsed.continuity.max_feed_ratio = ratio
                        parsed.continuity.feed_at = (seg.x0, seg.y0, seg.z)
                # Very short/tiny-E generated segments are dominated by G-code rounding
                # (5 decimal places in BOWP output) and produce meaningless ratios.
                ratio = flow_ratio(last_gen, last_len, last_density)
                if ratio is not None:
                    if ratio > parsed.continuity.max_flow_ratio:
                        parsed.continuity.max_flow_ratio = ratio
                        parsed.continuity.flow_at = (seg.x0, seg.y0, seg.z)
                parsed.continuity.pairs += 1
        if last_transition_gen is not None and last_transition_gen.kind != seg.kind:
            values = comparable(last_transition_gen)
            if values is not None:
                _, last_len, last_density = values
                if seg.feed > 0 and last_transition_gen.feed > 0:
                    ratio = max(seg.feed, last_transition_gen.feed) / max(1e-9, min(seg.feed, last_transition_gen.feed))
                    if ratio > parsed.continuity.max_transition_feed_ratio:
                        parsed.continuity.max_transition_feed_ratio = ratio
                        parsed.continuity.transition_feed_at = (seg.x0, seg.y0, seg.z)
                ratio = flow_ratio(last_transition_gen, last_len, last_density)
                if ratio is not None:
                    if ratio > parsed.continuity.max_transition_flow_ratio:
                        parsed.continuity.max_transition_flow_ratio = ratio
                        parsed.continuity.transition_flow_at = (seg.x0, seg.y0, seg.z)
                parsed.continuity.transition_pairs += 1
        last_gen = seg
        last_transition_gen = seg

    def append_extrusion(seg: Seg) -> None:
        parsed.cover[layer].append(seg)
        if seg.kind == "wall":
            parsed.wall[layer].append(seg)
            current.append(seg)
        elif seg.kind == "bowp":
            parsed.wall[layer].append(seg)
            parsed.finish[layer].append(seg)
            flush_loop()
        elif seg.kind == "iron":
            parsed.finish[layer].append(seg)
            flush_loop()
        else:
            flush_loop()
        update_continuity(seg)

    with open(path, encoding="utf-8", errors="ignore") as f:
        for raw in f:
            s = raw.strip()
            if not s:
                continue
            if s.startswith(";"):
                u = s.upper()
                if "LAYER_CHANGE" in u or "CHANGE_LAYER" in u or u.startswith(";LAYER:"):
                    flush_loop()
                    layer += 1
                    bowp = ""
                    section = ""
                    last_gen = None
                    last_transition_gen = None
                elif u.startswith("; BOWP") and " START" in u:
                    flush_loop()
                    bowp = "iron" if "SCRIPT IRONING" in u else "bowp"
                    last_gen = None
                elif u.startswith("; BOWP") and " END" in u:
                    bowp = ""
                    last_gen = None
                elif "TYPE:" in u or "FEATURE:" in u:
                    new_section = section
                    if "IRONING" in u:
                        new_section = "iron"
                    elif "OUTER WALL" in u or "EXTERNAL PERIMETER" in u:
                        new_section = "outer"
                    elif "OVERHANG" in u and (include_overhang_starts or section == "outer"):
                        new_section = "outer"
                    else:
                        new_section = "other"
                    if new_section != "outer":
                        flush_loop()
                    section = new_section
                continue
            code = s.split(";")[0].strip()
            if not code:
                continue
            cmd = code.split()[0].upper()
            if cmd == "M83":
                rel = True
                continue
            if cmd == "M82":
                rel = False
                continue
            if cmd == "G91":
                coord_rel = True
                continue
            if cmd == "G90":
                coord_rel = False
                continue
            if cmd == "G92":
                args = parse_tokens(code)
                if any(axis in args for axis in ("X", "Y", "Z")):
                    flush_loop()
                    last_gen = None
                    last_transition_gen = None
                if "X" in args:
                    x = args["X"]
                if "Y" in args:
                    y = args["Y"]
                if "Z" in args:
                    z = args["Z"]
                if "E" in args:
                    e = args["E"]
                continue
            if cmd not in ("G0", "G1", "G2", "G3"):
                continue
            args = parse_tokens(code)
            if "F" in args and args["F"] > 0:
                feed = args["F"]
            if coord_rel:
                nx = x + args.get("X", 0.0)
                ny = y + args.get("Y", 0.0)
                nz = z + args.get("Z", 0.0)
            else:
                nx, ny, nz = args.get("X", x), args.get("Y", y), args.get("Z", z)
            de = 0.0
            if "E" in args:
                de = args["E"] if rel else args["E"] - e
                if not rel:
                    e = args["E"]
            xy_len = math.hypot(nx - x, ny - y)
            if cmd in ("G2", "G3"):
                arc_probe = expand_arc_segments(cmd, args, x, y, z, nx, ny, nz, 0.0, feed, "other")
                if arc_probe:
                    xy_len = sum(math.hypot(seg.x1 - seg.x0, seg.y1 - seg.y0) for seg in arc_probe)
            z_len = abs(nz - z)
            if de <= 1e-9 and (xy_len > 1e-5 or z_len > 1e-5):
                last_gen = None
                last_transition_gen = None
            elif xy_len <= 1e-5 and z_len <= 1e-5 and abs(de) > 1e-9:
                last_gen = None
                last_transition_gen = None
            if xy_len > TRAVEL_GAP_MM and de <= 1e-9:
                flush_loop()
            if de > 1e-9 and xy_len > 1e-5 and layer >= 0:
                kind = "other"
                if bowp:
                    kind = bowp
                elif section == "iron":
                    kind = "iron"
                elif section == "outer":
                    kind = "wall"
                generated = bool(bowp)
                expanded = expand_arc_segments(cmd, args, x, y, z, nx, ny, nz, de, feed, kind, generated)
                if expanded:
                    for seg in expanded:
                        append_extrusion(seg)
                else:
                    append_extrusion(Seg(x, y, nx, ny, nz, de, feed, kind, generated))
            x, y, z = nx, ny, nz
    flush_loop()
    return parsed


def sample_segs(segs: list[Seg], max_pts: int) -> list[tuple[float, float]]:
    total = sum(math.hypot(seg.x1 - seg.x0, seg.y1 - seg.y0) for seg in segs)
    if total <= 0:
        return []
    step = max(0.4, total / max(1, max_pts))
    pts: list[tuple[float, float]] = []
    next_s = step * 0.5
    acc = 0.0
    for seg in segs:
        length = math.hypot(seg.x1 - seg.x0, seg.y1 - seg.y0)
        while next_s <= acc + length:
            t = (next_s - acc) / length if length > 1e-9 else 0.0
            pts.append((seg.x0 + (seg.x1 - seg.x0) * t, seg.y0 + (seg.y1 - seg.y0) * t))
            next_s += step
        acc += length
    return pts


def merge_intervals(intervals: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    ordered = sorted((min(a, b), max(a, b)) for a, b in intervals if abs(b - a) > 1e-9)
    if not ordered:
        return []
    out = [ordered[0]]
    for a, b in ordered[1:]:
        la, lb = out[-1]
        if a <= lb + 1e-9:
            out[-1] = (la, max(lb, b))
        else:
            out.append((a, b))
    return out


def subtract_intervals(base: list[tuple[float, float]], cuts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    out = merge_intervals(base)
    for ca, cb in merge_intervals(cuts):
        next_out: list[tuple[float, float]] = []
        for a, b in out:
            if cb <= a or ca >= b:
                next_out.append((a, b))
            else:
                if ca > a:
                    next_out.append((a, ca))
                if cb < b:
                    next_out.append((cb, b))
        out = next_out
        if not out:
            break
    return out


def scanline_grid(loop: Loop) -> dict[int, list[int]]:
    if loop.scanline_grid is not None:
        return loop.scanline_grid
    grid: dict[int, list[int]] = {}
    inv = 1.0 / SCANLINE_CELL_MM
    for idx, seg in enumerate(loop.segs):
        y0 = min(seg.y0, seg.y1)
        y1 = max(seg.y0, seg.y1)
        if y1 - y0 <= 1e-12:
            continue
        for gy in range(math.floor(y0 * inv), math.floor(y1 * inv) + 1):
            grid.setdefault(gy, []).append(idx)
    loop.scanline_grid = grid
    return grid


def loop_scanline_intervals(loop: Loop, y: float) -> list[tuple[float, float]]:
    if y < loop.y_min or y > loop.y_max:
        return []
    key = y
    cached = loop.interval_cache.get(key)
    if cached is not None:
        return cached
    xs: list[float] = []
    if len(loop.segs) <= 32:
        indices: Iterable[int] = range(len(loop.segs))
    else:
        indices = scanline_grid(loop).get(math.floor(y / SCANLINE_CELL_MM), ())
    for idx in indices:
        seg = loop.segs[idx]
        if (seg.y0 > y) == (seg.y1 > y):
            continue
        t = (y - seg.y0) / (seg.y1 - seg.y0)
        xs.append(seg.x0 + (seg.x1 - seg.x0) * t)
    xs.sort()
    intervals = [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2) if xs[i + 1] - xs[i] > 1e-9]
    loop.interval_cache[key] = intervals
    return intervals


def _material_intervals_for_loops(loops: Iterable[Loop], y: float) -> list[tuple[float, float]]:
    events: list[tuple[float, int]] = []
    for lp in loops:
        for a, b in loop_scanline_intervals(lp, y):
            events.append((a, 1))
            events.append((b, -1))
    if not events:
        return []
    events.sort()
    out: list[tuple[float, float]] = []
    depth = 0
    start: float | None = None
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


def _layered_material_intervals_for_loops(loops: Iterable[Loop], y: float) -> list[tuple[float, float]]:
    by_layer: dict[int, list[Loop]] = {}
    for lp in loops:
        if not loop_closed(lp.segs):
            continue
        by_layer.setdefault(lp.layer, []).append(lp)
    intervals: list[tuple[float, float]] = []
    for layer_loops in by_layer.values():
        intervals.extend(_material_intervals_for_loops(layer_loops, y))
    return merge_intervals(intervals)


def material_intervals(loops: list[Loop], y: float) -> list[tuple[float, float]]:
    return _layered_material_intervals_for_loops(loops, y)


class MaterialIndex:
    def __init__(self, loops: Iterable[Loop]) -> None:
        self.grid: dict[int, list[Loop]] = {}
        inv = 1.0 / SCANLINE_CELL_MM
        for lp in loops:
            if not loop_closed(lp.segs):
                continue
            for gy in range(math.floor(lp.y_min * inv), math.floor(lp.y_max * inv) + 1):
                self.grid.setdefault(gy, []).append(lp)

    def intervals(self, y: float) -> list[tuple[float, float]]:
        return _layered_material_intervals_for_loops(
            self.grid.get(math.floor(y / SCANLINE_CELL_MM), ()),
            y,
        )


def point_in_loop(px: float, py: float, loop: Loop) -> bool:
    inside = False
    for seg in loop.segs:
        if (seg.y0 > py) != (seg.y1 > py):
            t = (py - seg.y0) / (seg.y1 - seg.y0)
            if seg.x0 + (seg.x1 - seg.x0) * t > px:
                inside = not inside
    return inside


def point_in_material(px: float, py: float, loops: list[Loop]) -> bool:
    return any(a <= px <= b for a, b in material_intervals(loops, py))


def layer_bounds(loops: list[Loop]) -> tuple[float, float] | None:
    if not loops:
        return None
    return min(lp.y_min for lp in loops), max(lp.y_max for lp in loops)


def surface_samples(
    loops: list[Loop],
    next_loops: list[Loop],
    step: float,
    cap: int,
) -> list[tuple[float, float]]:
    bounds = layer_bounds(loops)
    if bounds is None:
        return []
    min_y, max_y = bounds
    rows: list[tuple[float, list[tuple[float, float]], int]] = []
    mandatory_rows: list[tuple[float, list[tuple[float, float]], int]] = []
    seen_rows: set[float] = set()
    estimate = 0
    current_index = MaterialIndex(loops)
    next_index = MaterialIndex(next_loops)
    y_marks: dict[float, int] = {}

    def mark_y(value: float, bit: int) -> None:
        if min_y - step <= value <= max_y + step:
            key = round(value, 5)
            y_marks[key] = y_marks.get(key, 0) | bit

    for lp in loops:
        for seg in lp.segs:
            mark_y(seg.y0, 1)
            mark_y(seg.y1, 1)
    for lp in next_loops:
        for seg in lp.segs:
            mark_y(seg.y0, 2)
            mark_y(seg.y1, 2)
    ordered_y = sorted(y_marks)

    def bare_at_y(y: float) -> list[tuple[float, float]]:
        return subtract_intervals(current_index.intervals(y), next_index.intervals(y))

    def stable_intervals(y: float, bare: list[tuple[float, float]]) -> list[tuple[float, float]]:
        probe = min(0.02, step * 0.05)
        near = merge_intervals(bare_at_y(y - probe) + bare_at_y(y + probe))
        stable: list[tuple[float, float]] = []
        min_overlap = min(0.1, step * 0.1)
        for a, b in bare:
            for na, nb in near:
                ia, ib = max(a, na), min(b, nb)
                if ib - ia >= min_overlap:
                    stable.append((ia, ib))
        return merge_intervals(stable)

    def near_boundary_y(y: float) -> bool:
        if not ordered_y:
            return False
        probe = min(0.02, step * 0.05)
        pos = bisect.bisect_left(ordered_y, y)
        if pos < len(ordered_y) and abs(ordered_y[pos] - y) <= probe:
            return True
        return pos > 0 and abs(ordered_y[pos - 1] - y) <= probe

    def row_weight(bare: list[tuple[float, float]]) -> int:
        return sum(max(1, int((b - a) / step)) for a, b in bare if b - a >= step * 0.5)

    def point_in_intervals(intervals: list[tuple[float, float]], x: float) -> bool:
        return any(a - 1e-9 <= x <= b + 1e-9 for a, b in intervals)

    def stable_current_point(x: float, y: float) -> bool:
        matched = False
        for lp in current_index.grid.get(math.floor(y / SCANLINE_CELL_MM), ()):
            in_interval = point_in_intervals(loop_scanline_intervals(lp, y), x)
            in_poly = point_in_loop(x, y, lp)
            if in_interval and in_poly:
                matched = True
                continue
            if in_interval != in_poly:
                return False
        return matched

    def add_row(y: float, mandatory: bool) -> None:
        nonlocal estimate
        if y < min_y or y > max_y:
            return
        key = round(y, 5)
        if key in seen_rows:
            return
        bare = bare_at_y(y)
        if mandatory or near_boundary_y(y):
            bare = stable_intervals(y, bare)
        n = row_weight(bare)
        if not n:
            return
        seen_rows.add(key)
        if mandatory:
            mandatory_rows.append((y, bare, n))
        else:
            rows.append((y, bare, n))
            estimate += n

    y = min_y + step * 0.5
    while y <= max_y - step * 0.25:
        add_row(y, False)
        y += step

    max_boundary_gap = min(step * 0.5, 0.6)
    max_mandatory_rows = max(16, cap // 3)
    for ya, yb in zip(ordered_y, ordered_y[1:]):
        if len(mandatory_rows) >= max_mandatory_rows:
            break
        if yb - ya <= 1e-5 or yb - ya > max_boundary_gap:
            continue
        if (y_marks[ya] & 1 and y_marks[yb] & 2) or (y_marks[ya] & 2 and y_marks[yb] & 1):
            add_row((ya + yb) * 0.5, True)

    if estimate <= 0 and not mandatory_rows:
        return []
    stride = max(1, math.ceil(estimate / max(1, cap)))
    pts: list[tuple[float, float]] = []

    def add_points(y: float, intervals: list[tuple[float, float]], use_stride: bool, counter: int) -> int:
        for a, b in intervals:
            if b - a < step * 0.5:
                continue
            n = max(1, int((b - a) / step))
            for k in range(n):
                x = a + (k + 0.5) * (b - a) / n
                if not stable_current_point(x, y):
                    counter += 1
                    continue
                # intervals are already current-material minus next-material at this scanline.
                if not use_stride or counter % stride == 0:
                    pts.append((x, y))
                counter += 1
        return counter

    counter = 0
    for y, intervals, _ in mandatory_rows:
        counter = add_points(y, intervals, False, counter)
    for y, intervals, _ in rows:
        counter = add_points(y, intervals, True, counter)
    return pts


def audit_wall(orig: Parsed, proc: Parsed, thr: float, per_layer: int) -> tuple[int, list[str]]:
    total_bad = 0
    reports: list[str] = []
    for ly in sorted(orig.wall):
        pts = sample_segs(orig.wall[ly], per_layer)
        if not pts:
            continue
        idx = SegIndex(proc.wall.get(ly, []))
        bad = [pt for pt in pts if idx.nearest(pt[0], pt[1], thr + CELL) > thr]
        if bad:
            total_bad += len(bad)
            ex = bad[0]
            reports.append(f"wall layer {ly}: {len(bad)}/{len(pts)} uncovered, e.g. ({ex[0]:.2f},{ex[1]:.2f})")
    return total_bad, reports


def z_loop_groups(loops_by_layer: dict[int, list[Loop]], tol: float = 0.02) -> list[tuple[list[int], list[Loop], float]]:
    items: list[tuple[float, int, Loop]] = []
    for ly, loops in loops_by_layer.items():
        for lp in loops:
            if lp.z_span > FLAT_LOOP_Z_TOL:
                continue
            items.append((lp.z, ly, lp))
    items.sort(key=lambda row: row[0])
    groups: list[tuple[list[int], list[Loop], float]] = []
    cur_layers: set[int] = set()
    cur_loops: list[Loop] = []
    cur_z: list[float] = []
    for z, ly, lp in items:
        if cur_z and z - cur_z[-1] > tol:
            groups.append((sorted(cur_layers), cur_loops, sum(cur_z) / len(cur_z)))
            cur_layers = set()
            cur_loops = []
            cur_z = []
        cur_layers.add(ly)
        cur_loops.append(lp)
        cur_z.append(z)
    if cur_z:
        groups.append((sorted(cur_layers), cur_loops, sum(cur_z) / len(cur_z)))
    return groups


def collect_layer_segs(segs_by_layer: dict[int, list[Seg]], layers: Iterable[int]) -> list[Seg]:
    out: list[Seg] = []
    for ly in layers:
        out.extend(segs_by_layer.get(ly, ()))
    return out


@dataclass(slots=True)
class GeneratedSafety:
    external_bad: int = 0
    internal_bad: int = 0
    unneeded_ironing_bad: int = 0
    samples: int = 0
    generated_segments: int = 0
    reports: list[str] = field(default_factory=list)


def layer_xy_bounds(loops: list[Loop]) -> tuple[float, float, float, float] | None:
    if not loops:
        return None
    xs = [v for lp in loops for seg in lp.segs for v in (seg.x0, seg.x1)]
    ys = [v for lp in loops for seg in lp.segs for v in (seg.y0, seg.y1)]
    if not xs or not ys:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def interval_contains_tol(intervals: list[tuple[float, float]], x: float, tol: float) -> bool:
    return any(a - tol <= x <= b + tol for a, b in intervals)


def point_in_index_material(
    indexes: dict[int, MaterialIndex],
    layer: int,
    x: float,
    y: float,
    tol: float,
) -> bool:
    idx = indexes.get(layer)
    return idx is not None and interval_contains_tol(idx.intervals(y), x, tol)


def next_layers_by_z(layer_z: dict[int, float]) -> dict[int, list[int]]:
    ordered = sorted(layer_z.items(), key=lambda item: item[1])
    out: dict[int, list[int]] = {}
    for i, (layer, z_val) in enumerate(ordered):
        first_dz: float | None = None
        nxt: list[int] = []
        for other, other_z in ordered[i + 1:]:
            dz = other_z - z_val
            if dz > NEIGHBOR_DZ:
                break
            if dz > 0.02:
                if first_dz is None:
                    first_dz = dz
                if abs(dz - first_dz) <= 0.02:
                    nxt.append(other)
                else:
                    break
        out[layer] = nxt
    return out


def nearest_layers_for_z(layer_z: dict[int, float], z: float) -> list[int]:
    if not layer_z:
        return []
    best = min(abs(z_val - z) for z_val in layer_z.values())
    return [
        layer
        for layer, z_val in layer_z.items()
        if abs(abs(z_val - z) - best) <= 0.02
    ]


def point_on_bare_surface(
    indexes: dict[int, MaterialIndex],
    next_by_layer: dict[int, list[int]],
    candidates: Iterable[int],
    x: float,
    y: float,
    tol: float,
) -> bool:
    for layer in candidates:
        if not point_in_index_material(indexes, layer, x, y, tol):
            continue
        covered_above = any(
            point_in_index_material(indexes, nxt, x, y, tol)
            for nxt in next_by_layer.get(layer, ())
        )
        if not covered_above:
            return True
    return False


def classify_material_point(
    indexes: dict[int, MaterialIndex],
    seg_indexes: dict[int, SegIndex],
    bounds: dict[int, tuple[float, float, float, float]],
    candidates: Iterable[int],
    x: float,
    y: float,
    tol: float,
) -> str:
    has_near_envelope = False
    for ly in candidates:
        idx = indexes.get(ly)
        if idx is None:
            continue
        seg_idx = seg_indexes.get(ly)
        if seg_idx is not None and seg_idx.nearest(x, y, tol + CELL) <= tol:
            return "ok"
        intervals = idx.intervals(y)
        if interval_contains_tol(intervals, x, tol):
            return "ok"
        if intervals:
            min_x = min(a for a, _ in intervals)
            max_x = max(b for _, b in intervals)
            if min_x - tol <= x <= max_x + tol:
                has_near_envelope = True
        bbox = bounds.get(ly)
        if bbox is not None:
            bx0, by0, bx1, by1 = bbox
            if bx0 - tol <= x <= bx1 + tol and by0 - tol <= y <= by1 + tol:
                has_near_envelope = True
    return "internal" if has_near_envelope else "external"


def segment_sample_points(seg: Seg, step: float) -> list[tuple[float, float]]:
    length = math.hypot(seg.x1 - seg.x0, seg.y1 - seg.y0)
    if length <= 1e-9:
        return [((seg.x0 + seg.x1) * 0.5, (seg.y0 + seg.y1) * 0.5)]
    n = max(1, math.ceil(length / max(0.1, step)))
    pts: list[tuple[float, float]] = []
    for i in range(n):
        t = (i + 0.5) / n
        pts.append((seg.x0 + (seg.x1 - seg.x0) * t, seg.y0 + (seg.y1 - seg.y0) * t))
    return pts


def audit_generated_safety(orig: Parsed, proc: Parsed, tol: float, sample_step: float) -> GeneratedSafety:
    indexes = {ly: MaterialIndex(loops) for ly, loops in orig.loops.items()}
    seg_indexes = {ly: SegIndex(segs) for ly, segs in orig.cover.items() if segs}
    wall_indexes = {ly: SegIndex(segs) for ly, segs in orig.wall.items() if segs}
    layer_z = {
        ly: sum(lp.z for lp in loops) / len(loops)
        for ly, loops in orig.loops.items()
        if loops
    }
    bounds = {
        ly: bbox
        for ly, loops in orig.loops.items()
        if (bbox := layer_xy_bounds(loops)) is not None
    }
    next_by_layer = next_layers_by_z(layer_z)
    topology_tol = min(0.25, max(0.03, tol * 0.25))
    result = GeneratedSafety()
    candidate_cache: dict[int, list[int]] = {}
    surface_layer_cache: dict[int, list[int]] = {}
    for ly in sorted(proc.cover):
        for seg in proc.cover[ly]:
            if not seg.generated or seg.kind not in {"bowp", "iron"}:
                continue
            result.generated_segments += 1
            z_key = int(round(seg.z * 1000))
            candidates = candidate_cache.get(z_key)
            if candidates is None:
                candidates = [
                    candidate
                    for candidate, z_val in layer_z.items()
                    if abs(z_val - seg.z) <= NEIGHBOR_DZ
                ]
                if not candidates:
                    candidates = [ly - 2, ly - 1, ly, ly + 1, ly + 2]
                candidate_cache[z_key] = candidates
            for px, py in segment_sample_points(seg, sample_step):
                result.samples += 1
                cls = classify_material_point(indexes, seg_indexes, bounds, candidates, px, py, tol)
                if cls == "ok":
                    surface_layers = surface_layer_cache.get(z_key)
                    if surface_layers is None:
                        surface_layers = nearest_layers_for_z(layer_z, seg.z)
                        surface_layer_cache[z_key] = surface_layers
                    near_wall = any(
                        wall_indexes.get(layer) is not None
                        and wall_indexes[layer].nearest(px, py, tol + CELL) <= tol
                        for layer in surface_layers
                    )
                    if (
                        seg.kind == "iron"
                        and not near_wall
                        and not point_on_bare_surface(
                            indexes,
                            next_by_layer,
                            surface_layers,
                            px,
                            py,
                            topology_tol,
                        )
                    ):
                        result.unneeded_ironing_bad += 1
                        if len(result.reports) < 30:
                            result.reports.append(
                                f"generated unneeded ironing layer {ly} z={seg.z:.3f}: "
                                f"({px:.2f},{py:.2f})"
                            )
                    continue
                if cls == "external":
                    result.external_bad += 1
                else:
                    result.internal_bad += 1
                if len(result.reports) < 30:
                    result.reports.append(
                        f"generated {cls} layer {ly} z={seg.z:.3f}: "
                        f"({px:.2f},{py:.2f}) kind={seg.kind}"
                    )
    return result


def audit_surface(
    orig: Parsed,
    proc: Parsed,
    thr: float,
    step: float,
    per_layer: int,
    require_top_finish: bool,
    pass_radius: float | None = None,
) -> tuple[int, int, list[str], PassStats]:
    pass_radius = thr if pass_radius is None else max(0.05, pass_radius)
    groups = z_loop_groups(orig.loops)
    total_bad = 0
    finish_missing = 0
    reports: list[str] = []
    pass_min = 10**9
    pass_max = 0
    pass_sum = 0
    pass_sq_sum = 0
    pass_n = 0
    for i, (layers, loops, z_val) in enumerate(groups):
        current_area = sum(abs(lp.area) for lp in loops)
        next_layers: list[int] = []
        next_loops: list[Loop] = []
        for future_layers, future_loops, future_z in groups[i + 1:]:
            dz = future_z - z_val
            if dz <= 0:
                continue
            if dz > NEIGHBOR_DZ:
                break
            next_layers.extend(future_layers)
            next_loops.extend(future_loops)
            future_area = sum(abs(lp.area) for lp in future_loops)
            if current_area <= 1e-9 or future_area >= current_area * 0.25:
                break
        pts = surface_samples(loops, next_loops, step, per_layer)
        if not pts:
            continue
        cover = collect_layer_segs(proc.cover, layers)
        cover.extend(collect_layer_segs(proc.cover, next_layers))
        finish = collect_layer_segs(proc.finish, layers)
        finish.extend(collect_layer_segs(proc.finish, next_layers))
        cover_idx = SegIndex(cover)
        finish_idx = SegIndex(finish)
        layer_bad = []
        layer_finish_missing = []
        for px, py in pts:
            d = cover_idx.nearest(px, py, thr + CELL)
            if d > thr:
                layer_bad.append((px, py))
            hits = cover_idx.pass_count_within(px, py, pass_radius)
            pass_min = min(pass_min, hits)
            pass_max = max(pass_max, hits)
            pass_sum += hits
            pass_sq_sum += hits * hits
            pass_n += 1
            if not next_layers and finish_idx.nearest(px, py, thr + CELL) > thr:
                layer_finish_missing.append((px, py))
        if layer_bad:
            total_bad += len(layer_bad)
            ex = layer_bad[0]
            reports.append(
                f"surface layer {layers[0]} z={z_val:.3f}: {len(layer_bad)}/{len(pts)} uncovered, "
                f"e.g. ({ex[0]:.2f},{ex[1]:.2f})"
            )
        if layer_finish_missing:
            finish_missing += len(layer_finish_missing)
            if require_top_finish:
                ex = layer_finish_missing[0]
                reports.append(
                    f"top finish layer {layers[0]} z={z_val:.3f}: "
                    f"{len(layer_finish_missing)}/{len(pts)} without ironing/spiral, "
                    f"e.g. ({ex[0]:.2f},{ex[1]:.2f})"
                )
    avg = pass_sum / pass_n if pass_n else 0.0
    variance = max(0.0, pass_sq_sum / pass_n - avg * avg) if pass_n else 0.0
    pass_stats = PassStats(
        min_count=0 if pass_min == 10**9 else pass_min,
        max_count=pass_max,
        avg=avg,
        stdev=math.sqrt(variance),
        samples=pass_n,
    )
    return total_bad, finish_missing, reports, pass_stats


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Audit BOWP G-code coverage and generated-path continuity.")
    parser.add_argument("original")
    parser.add_argument("processed")
    parser.add_argument("--thr", type=float, default=None, help="Back-compat alias for both wall and surface thresholds")
    parser.add_argument("--wall-thr", type=float, default=0.9)
    parser.add_argument("--surface-thr", type=float, default=0.6)
    parser.add_argument("--generated-thr", type=float, default=0.9)
    parser.add_argument("--generated-step", type=float, default=0.8)
    parser.add_argument("--max-generated-external", type=int, default=0)
    parser.add_argument(
        "--max-generated-internal",
        type=int,
        default=-1,
        help="Optional failure threshold for generated samples in internal voids; -1 reports only",
    )
    parser.add_argument(
        "--max-generated-unneeded-ironing",
        type=int,
        default=0,
        help="Failure threshold for generated script-ironing samples that are not on exposed top surface",
    )
    parser.add_argument("--per-layer", type=int, default=300, help="Back-compat wall sample cap")
    parser.add_argument("--per-layer-surface", type=int, default=1500)
    parser.add_argument("--surface-step", type=float, default=0.8)
    parser.add_argument(
        "--pass-radius",
        type=float,
        default=0.25,
        help="Radius for pass-count uniformity statistics; coverage still uses --surface-thr",
    )
    parser.add_argument("--max-feed-ratio", type=float, default=4.0)
    parser.add_argument("--max-flow-ratio", type=float, default=8.0)
    parser.add_argument("--max-transition-feed-ratio", type=float, default=4.0)
    parser.add_argument("--max-transition-flow-ratio", type=float, default=8.0)
    parser.add_argument(
        "--include-overhang-wall",
        action="store_true",
        help="Treat overhang-wall sections as outer wall starts, matching --max-overhang -90 processing",
    )
    parser.add_argument(
        "--max-pass-cv",
        type=float,
        default=0.0,
        help="Optional failure threshold for pass-count coefficient of variation; 0 disables",
    )
    parser.add_argument("--require-top-finish", action="store_true")
    args = parser.parse_args(argv)

    if args.thr is not None:
        args.wall_thr = args.thr
        args.surface_thr = args.thr
        args.generated_thr = args.thr

    orig = parse_file(args.original, args.include_overhang_wall)
    proc = parse_file(args.processed, args.include_overhang_wall)
    wall_bad, wall_reports = audit_wall(orig, proc, args.wall_thr, args.per_layer)
    gen_safety = audit_generated_safety(
        orig,
        proc,
        args.generated_thr,
        max(0.1, args.generated_step),
    )
    surf_bad, finish_missing, surf_reports, pass_stats = audit_surface(
        orig,
        proc,
        args.surface_thr,
        max(0.1, args.surface_step),
        args.per_layer_surface,
        args.require_top_finish,
        args.pass_radius,
    )

    cont_bad = []
    if proc.continuity.max_feed_ratio > args.max_feed_ratio:
        cont_bad.append(f"feed ratio {proc.continuity.max_feed_ratio:.2f} at {proc.continuity.feed_at}")
    if proc.continuity.max_flow_ratio > args.max_flow_ratio:
        cont_bad.append(f"flow ratio {proc.continuity.max_flow_ratio:.2f} at {proc.continuity.flow_at}")
    if (
        proc.continuity.transition_pairs
        and proc.continuity.max_transition_feed_ratio > args.max_transition_feed_ratio
    ):
        cont_bad.append(
            f"transition feed ratio {proc.continuity.max_transition_feed_ratio:.2f} "
            f"at {proc.continuity.transition_feed_at}"
        )
    if (
        proc.continuity.transition_pairs
        and proc.continuity.max_transition_flow_ratio > args.max_transition_flow_ratio
    ):
        cont_bad.append(
            f"transition flow ratio {proc.continuity.max_transition_flow_ratio:.2f} "
            f"at {proc.continuity.transition_flow_at}"
        )
    if args.max_pass_cv > 0 and pass_stats.cv > args.max_pass_cv:
        cont_bad.append(f"pass-count cv {pass_stats.cv:.2f} > {args.max_pass_cv:.2f}")

    gen_bad = []
    if gen_safety.external_bad > args.max_generated_external:
        gen_bad.append(
            f"generated external {gen_safety.external_bad} > {args.max_generated_external}"
        )
    if args.max_generated_internal >= 0 and gen_safety.internal_bad > args.max_generated_internal:
        gen_bad.append(
            f"generated internal {gen_safety.internal_bad} > {args.max_generated_internal}"
        )
    if gen_safety.unneeded_ironing_bad > args.max_generated_unneeded_ironing:
        gen_bad.append(
            "generated unneeded ironing "
            f"{gen_safety.unneeded_ironing_bad} > {args.max_generated_unneeded_ironing}"
        )

    fail = (
        wall_bad
        or surf_bad
        or cont_bad
        or gen_bad
        or (args.require_top_finish and finish_missing)
    )
    if fail:
        print(
            f"COVERAGE_FAIL wall_bad={wall_bad} surface_bad={surf_bad} "
            f"top_finish_missing={finish_missing} "
            f"generated_external_bad={gen_safety.external_bad} "
            f"generated_internal_bad={gen_safety.internal_bad} "
            f"generated_unneeded_ironing_bad={gen_safety.unneeded_ironing_bad}"
        )
        for line in (wall_reports + surf_reports)[:30]:
            print("  " + line)
        for line in gen_safety.reports[:30]:
            print("  " + line)
        for line in cont_bad:
            print("  continuity " + line)
        for line in gen_bad:
            print("  safety " + line)
        print(
            f"pass_count min={pass_stats.min_count} max={pass_stats.max_count} "
            f"avg={pass_stats.avg:.2f} stdev={pass_stats.stdev:.2f} cv={pass_stats.cv:.2f} "
            f"samples={pass_stats.samples} radius={args.pass_radius:.2f}; "
            f"generated_safety samples={gen_safety.samples} "
            f"segments={gen_safety.generated_segments} "
            f"external_bad={gen_safety.external_bad} "
            f"internal_bad={gen_safety.internal_bad} "
            f"unneeded_ironing_bad={gen_safety.unneeded_ironing_bad}; "
            f"generated continuity pairs={proc.continuity.pairs} "
            f"feed_ratio={proc.continuity.max_feed_ratio:.2f} "
            f"flow_ratio={proc.continuity.max_flow_ratio:.2f} "
            f"transition_pairs={proc.continuity.transition_pairs} "
            f"transition_feed_ratio={proc.continuity.max_transition_feed_ratio:.2f} "
            f"transition_flow_ratio={proc.continuity.max_transition_flow_ratio:.2f}"
        )
        return 1

    print(
        f"coverage_ok layers={len(orig.loops)} wall_thr={args.wall_thr} surface_thr={args.surface_thr} "
        f"top_finish_missing={finish_missing} "
        f"generated_external_bad={gen_safety.external_bad} "
        f"generated_internal_bad={gen_safety.internal_bad} "
        f"generated_unneeded_ironing_bad={gen_safety.unneeded_ironing_bad}"
    )
    print(
        f"pass_count min={pass_stats.min_count} max={pass_stats.max_count} "
        f"avg={pass_stats.avg:.2f} stdev={pass_stats.stdev:.2f} cv={pass_stats.cv:.2f} "
        f"samples={pass_stats.samples} radius={args.pass_radius:.2f}; "
        f"generated_safety samples={gen_safety.samples} "
        f"segments={gen_safety.generated_segments} "
        f"external_bad={gen_safety.external_bad} "
        f"internal_bad={gen_safety.internal_bad} "
        f"unneeded_ironing_bad={gen_safety.unneeded_ironing_bad}; "
        f"generated continuity pairs={proc.continuity.pairs} "
        f"feed_ratio={proc.continuity.max_feed_ratio:.2f} "
        f"flow_ratio={proc.continuity.max_flow_ratio:.2f} "
        f"transition_pairs={proc.continuity.transition_pairs} "
        f"transition_feed_ratio={proc.continuity.max_transition_feed_ratio:.2f} "
        f"transition_flow_ratio={proc.continuity.max_transition_flow_ratio:.2f}"
    )
    if finish_missing:
        print("warning: top surface has samples without ironing/spiral finishing; use --require-top-finish to fail this")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
