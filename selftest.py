from __future__ import annotations

import math
import subprocess
import shutil
import sys
import tempfile
from pathlib import Path

import audit_coverage
import better_outer_wall_processing as plugin
import validate_gcode


def assert_contains(text: str, value: str) -> None:
    if value not in text:
        raise AssertionError(f"missing expected text: {value!r}")


def assert_close(actual: float, expected: float, tol: float, label: str) -> None:
    if abs(actual - expected) > tol:
        raise AssertionError(f"{label}: expected {expected:.5f}, got {actual:.5f} (tol {tol})")


def simulate(path: Path) -> dict:
    """独立重放输出 G-code：统计净挤出、回抽幅度、挤出移动跳变。"""
    x = y = z = 0.0
    e = 0.0
    relative = False
    coord_relative = False
    net_ext = 0.0
    max_retract = 0.0
    max_xy_jump_extruding = 0.0
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = plugin.parse_line(raw + "\n")
        if line.command == "M83":
            relative = True
        elif line.command == "M82":
            relative = False
        elif line.command == "G91":
            coord_relative = True
        elif line.command == "G90":
            coord_relative = False
        elif line.command == "G92":
            if "X" in line.args:
                x = line.args["X"]
            if "Y" in line.args:
                y = line.args["Y"]
            if "Z" in line.args:
                z = line.args["Z"]
            if "E" in line.args:
                e = line.args["E"]
        elif line.command in ("G0", "G1"):
            if coord_relative:
                nx = x + line.args.get("X", 0.0)
                ny = y + line.args.get("Y", 0.0)
                nz = z + line.args.get("Z", 0.0)
            else:
                nx = line.args.get("X", x)
                ny = line.args.get("Y", y)
                nz = line.args.get("Z", z)
            de = 0.0
            if "E" in line.args:
                de = line.args["E"] if relative else line.args["E"] - e
                if not relative:
                    e = line.args["E"]
            if de > 0:
                max_xy_jump_extruding = max(max_xy_jump_extruding, math.hypot(nx - x, ny - y))
            elif de < 0:
                max_retract = max(max_retract, -de)
            net_ext += de
            x, y, z = nx, ny, nz
    return {
        "net_extrusion": net_ext,
        "max_retract": max_retract,
        "max_xy_jump_extruding": max_xy_jump_extruding,
    }


def expected_net_extrusion(input_path: Path, cfg: dict) -> float:
    """独立推导预期净挤出。

    螺旋/斜接缝模式：非短墙回路总挤出 = loop_ext * primary_flow（精确守恒）。
    平面模式：接缝渐变少挤的部分不补回，按 segment_scale 累计。"""
    lines = [plugin.parse_line(raw) for raw in input_path.read_text(encoding="utf-8").splitlines(keepends=True)]
    infos, _, _, _ = plugin.annotate(lines)
    loops, _ = plugin.build_loops(infos)
    total = sum(info.de for info in infos)
    scarf = float(cfg["scarf_length_mm"])
    min_wall = max(float(cfg["min_wall_length_mm"]), 2 * scarf)
    wall_limit = float(cfg.get("wall_length_limit_mm", 1.0))
    fallback = min(0.45, float(cfg.get("short_wall_fallback_rel", 0.25)))
    base = float(cfg["primary_flow_scale"])
    mode = cfg.get("seam_mode", "spiral")
    for loop in loops:
        loop_ext = sum(seg.de for seg in loop.segments)
        # 首层（层高未知）退化为 flat；最顶层仅 scarf 退化
        # （spiral 照常处理，插值下半程拉伸到整层）
        if not _loop_has_height(loop, loops):
            eff_mode = "flat"
        elif mode == "scarf" and not _loop_has_next(loop, loops):
            eff_mode = "flat"
        else:
            eff_mode = mode
        processed = loop.total_length >= wall_limit
        eff_cfg = cfg
        if processed and loop.total_length < min_wall:
            eff_cfg = dict(cfg, scarf_length_mm=loop.total_length * fallback)
        if processed:
            if eff_mode in ("spiral", "scarf"):
                # 螺旋接缝补偿为再分配式（减料全额摊回 payback 段），
                # 净挤出精确守恒
                total -= loop_ext * (1.0 - base)
            else:
                sample_step = max(0.05, float(cfg.get("sample_step_mm", 1.0)))
                sub = plugin.subdivide_loop(loop, plugin.generated_sample_step(loop, eff_cfg, sample_step))
                traveled = 0.0
                scaled = 0.0
                for seg in sub.segments:
                    mid = traveled + seg.length * 0.5
                    scaled += seg.de * plugin.segment_scale(mid, sub.total_length, eff_cfg)
                    traveled += seg.length
                total -= loop_ext - scaled
        if cfg.get("secondary_pass_enabled", True):
            ramp = min(2.0, loop.total_length * 0.15)
            # 所有模式首尾都有流量渐变，净少一个 ramp 长度
            eff = max(0.0, loop.total_length - ramp) / loop.total_length
            total += loop_ext * float(cfg["secondary_flow_scale"]) * eff
    return total


def _loop_has_height(loop, loops) -> bool:
    zs = sorted({round(lp.z, 6) for lp in loops})
    return zs.index(round(loop.z, 6)) > 0


def _loop_has_next(loop, loops) -> bool:
    zs = sorted({round(lp.z, 6) for lp in loops})
    return zs.index(round(loop.z, 6)) < len(zs) - 1


def check_processed(input_path: Path, cfg: dict) -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    try:
        work = temp_dir / input_path.name
        shutil.copyfile(input_path, work)
        seam_count, stats = plugin.run_processor(str(work), cfg)
        text = work.read_text(encoding="utf-8")

        assert seam_count == 3, f"expected 3 seam loops, got {seam_count}"
        if cfg.get("secondary_pass_enabled", True):
            assert_contains(text, "BOWP secondary pass start")
        mode = cfg.get("seam_mode", "spiral")
        if mode == "spiral":
            assert_contains(text, "BOWP spiral start")
            assert_contains(text, "BOWP spiral top fill")
            if cfg.get("spiral_flatten_enabled", True):
                assert_contains(text, "BOWP spiral flatten")
            if float(cfg.get("purge_retract_mm", 0.0)) > 0:
                assert_contains(text, "BOWP purge lap start")
                assert_contains(text, "BOWP purge lap end")
        elif mode == "scarf":
            assert_contains(text, "BOWP scarf overlap start")

        replay = simulate(work)
        exp = expected_net_extrusion(input_path, cfg)
        # XY 插值使螺旋实际周长与原轮廓略有差异，流量按实际长度给
        # （线密度恒定），允许 0.2% 相对偏差
        assert_close(replay["net_extrusion"], exp, max(1e-3, 0.002 * abs(exp)), "extrusion conservation")
        assert stats.final_time_s > 0

        assert replay["max_retract"] <= 0.9, f"unexpected retract {replay['max_retract']}"
        assert replay["max_xy_jump_extruding"] <= 20.1, f"XY jump {replay['max_xy_jump_extruding']}"

        # 幂等性
        before = text
        seam2, _ = plugin.run_processor(str(work), cfg)
        assert seam2 == 0, "second run should be a no-op"
        assert work.read_text(encoding="utf-8") == before, "second run modified the file"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_spiral_geometry(input_path: Path, cfg: dict) -> None:
    """螺旋几何：第二层（z=0.4，h=0.2）挤出 Z 应从 0.2 单调爬升到 0.4，
    共 x 圈；压平圈 Z 恒定于 0.4 且零挤出。"""
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    try:
        work = temp_dir / "spiral.gcode"
        shutil.copyfile(input_path, work)
        plugin.run_processor(str(work), cfg)
        lines = work.read_text(encoding="utf-8").splitlines()
        x = int(cfg["seam_superres_x"])

        # 首层（层高未知）退化为 flat；第二层与最顶层均 spiral
        # （最顶层插值下半程拉伸到整层）。几何检查用第二层（完整规则）
        starts = [i for i, l in enumerate(lines) if l.startswith(f"; BOWP spiral start x{x}")]
        assert len(starts) == 2, f"expected 2 spiral blocks, got {len(starts)}"
        begin = starts[0]
        end = next(i for i in range(begin, len(lines)) if lines[i].startswith("; BOWP spiral end"))
        topfill = next(i for i in range(begin, end) if lines[i].startswith("; BOWP spiral top fill"))

        e = 0.0
        for raw in lines[:begin]:
            line = plugin.parse_line(raw + "\n")
            if line.command == "G92" and "E" in line.args:
                e = line.args["E"]
            elif line.command in ("G0", "G1") and "E" in line.args:
                e = line.args["E"]

        # 螺旋段：Z 单调不减，从 ~0.2 到 0.4
        zs = []
        for raw in lines[begin:topfill]:
            line = plugin.parse_line(raw + "\n")
            if line.command == "G1" and "E" in line.args and "Z" in line.args:
                zs.append(line.args["Z"])
        assert zs, "spiral block has no extrusion"
        assert zs == sorted(zs), f"spiral Z not monotonic: {zs[:8]}..."
        assert zs[0] > 0.2 and abs(zs[-1] - 0.4) < 1e-6, f"spiral Z range wrong: {zs[0]}..{zs[-1]}"
        # 每圈段数 = 细分后的段数（20 边长 / 1mm 步长 * 4 边 = 80 段/圈）
        assert len(zs) >= 4 * x, f"expected >= {4*x} spiral segments, got {len(zs)}"

        # 压平圈：Z 恒 0.4、E 不变
        if cfg.get("spiral_flatten_enabled", True):
            flat_begin = next(i for i in range(begin, end) if lines[i].startswith("; BOWP spiral flatten"))
            prev_e = None
            for raw in lines[flat_begin:end]:
                line = plugin.parse_line(raw + "\n")
                if line.command == "G1" and "X" in line.args:
                    assert abs(line.args.get("Z", 0.4) - 0.4) < 1e-6, f"flatten Z wrong: {raw}"
                    if "E" in line.args and prev_e is not None:
                        assert abs(line.args["E"] - prev_e) < 1e-9, f"flatten extrudes: {raw}"
                    if "E" in line.args:
                        prev_e = line.args["E"]
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_config_roundtrip() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    orig_config_path = plugin.CONFIG_PATH
    orig_log_path = plugin.LOG_PATH
    try:
        plugin.CONFIG_PATH = str(temp_dir / "config.json")
        plugin.LOG_PATH = str(temp_dir / "progress.log")
        custom = dict(plugin.DEFAULT_CONFIG)
        custom["enabled"] = False
        custom["seam_mode"] = "flat"
        custom["seam_superres_x"] = 7
        custom["keep_processed_copy"] = True
        plugin.save_config(custom)
        loaded = plugin.load_config()
        assert loaded["enabled"] is False
        assert loaded["seam_mode"] == "flat"
        assert loaded["seam_superres_x"] == 7
        assert loaded["keep_processed_copy"] is True
        assert plugin.coerce_config_value("bool", 1) is True
        assert plugin.coerce_config_value("int", "4") == 4
        assert abs(plugin.coerce_config_value("float", "1.25") - 1.25) < 1e-9
        assert plugin.coerce_config_value("enum", "spiral") == "spiral"
    finally:
        plugin.CONFIG_PATH = orig_config_path
        plugin.LOG_PATH = orig_log_path
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_audit_coverage_script(base: Path) -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-audit-selftest-"))
    try:
        work = temp_dir / "sample_input.gcode"
        shutil.copyfile(base / "sample_input.gcode", work)
        plugin.run_processor(str(work), dict(
            plugin.DEFAULT_CONFIG,
            progress_enabled=False,
            secondary_pass_enabled=False,
            purge_retract_mm=0.0,
        ))
        proc = subprocess.run(
            [
                sys.executable,
                str(base / "audit_coverage.py"),
                str(base / "sample_input.gcode"),
                str(work),
                "--wall-thr", "0.9",
                "--surface-thr", "0.9",
                "--per-layer", "80",
                "--per-layer-surface", "120",
                "--surface-step", "2.0",
                "--require-top-finish",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        assert_contains(proc.stdout, "coverage_ok")
        assert_contains(proc.stdout, "pass_count")
        assert_contains(proc.stdout, "cv=")
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_validate_gcode_script(base: Path) -> None:
    parsed_args = validate_gcode.processor_args_from_command_text(
        '"C:\\path\\to\\better-outer-wall-processing\\run_postprocess.bat" --seam-superres 3 --disable-secondary'
    )
    assert parsed_args == ["--seam-superres", "3", "--disable-secondary"], parsed_args
    proc = subprocess.run(
        [
            sys.executable,
            str(base / "validate_gcode.py"),
            str(base / "sample_input.gcode"),
            "--wall-thr", "0.9",
            "--surface-thr", "0.9",
            "--per-layer", "80",
            "--per-layer-surface", "120",
            "--surface-step", "2.0",
            "--processor-args", "--seam-superres 3 --disable-secondary --purge-retract 0",
            "--require-top-finish",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    out = proc.stdout
    for key in [
        "status=ok",
        "wall_bad=0",
        "surface_bad=0",
        "top_finish_missing=0",
        "marker_scan_full=",
        "processor_args_count=",
        "audit_include_overhang_wall=",
        "legacy_step_ironing=",
        "current_script_ironing=",
        "spiral_markers=",
        "generated_external_bad=0",
        "generated_internal_bad=0",
        "generated_unneeded_ironing_bad=0",
        "generated_safety_samples=",
        "generated_safety_segments=",
        "pass_count_min=",
        "pass_count_max=",
        "pass_count_avg=",
        "pass_count_stdev=",
        "pass_count_cv=",
        "pass_count_samples=",
        "pass_count_radius=",
        "feed_ratio=",
        "flow_ratio=",
        "transition_pairs=",
        "transition_feed_ratio=",
        "transition_flow_ratio=",
        "batch_summary=begin",
        "batch_total=",
        "batch_ok=",
        "batch_fail=",
        "batch_skipped_already_processed=",
        "batch_recursive=",
        "batch_process_s=",
        "batch_audit_s=",
        "batch_wall_bad=",
        "batch_surface_bad=",
        "batch_top_finish_missing=",
        "batch_generated_external_bad=",
        "batch_generated_internal_bad=",
        "batch_generated_unneeded_ironing_bad=",
        "batch_generated_safety_samples=",
        "batch_script_ironing_lines=",
        "batch_max_feed_ratio=",
        "batch_max_flow_ratio=",
        "batch_max_transition_feed_ratio=",
        "batch_max_transition_flow_ratio=",
        "batch_max_pass_cv=",
        "batch_summary=end",
    ]:
        assert_contains(out, key)


def check_validate_recursive_discovery() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-discover-"))
    try:
        root_file = temp_dir / "root.gcode"
        nested_dir = temp_dir / "nested"
        nested_dir.mkdir()
        nested_file = nested_dir / "nested.gcode"
        root_file.write_text("; root\n", encoding="utf-8")
        nested_file.write_text("; nested\nG1 X0 Y0\n", encoding="utf-8")
        (nested_dir / "ignore.stl").write_text("solid x\nendsolid x\n", encoding="utf-8")

        flat = validate_gcode.discover_files(temp_dir, None, None, None)
        recursive = validate_gcode.discover_files(temp_dir, None, None, None, recursive=True)
        assert flat == [root_file], f"flat discovery should only include root file: {flat}"
        assert recursive == [root_file, nested_file], f"recursive discovery missed nested file: {recursive}"
        limited = validate_gcode.discover_files(temp_dir, 1, None, None, recursive=True)
        assert limited == [root_file], f"recursive limit should preserve sorted order: {limited}"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_scanline_index_consistency() -> None:
    interval_cases = [
        ([], []),
        ([(0.0, 2.0)], [(1.0, 3.0)]),
        ([(0.0, 5.0)], [(0.5, 1.0), (2.0, 3.0), (6.0, 7.0)]),
        ([(0.0, 1.0), (2.0, 4.0)], [(0.5, 3.0)]),
        ([(0.0, 1.0), (2.0, 4.0)], [(0.5, 2.5), (3.0, 5.0)]),
    ]
    for left, right in interval_cases:
        expected = plugin.intersect_intervals(left, right)
        assert plugin.intersect_sorted_intervals(left, right) == expected
        for third in ([], [(0.25, 0.75)], [(0.0, 1.5), (2.5, 3.5)]):
            expected_three = plugin.intersect_sorted_intervals(expected, third)
            assert plugin.intersect_three_sorted_intervals(left, right, third) == expected_three

    def subtract_reference(intervals, cuts):
        out = list(intervals)
        for ca, cb in plugin.merge_intervals(cuts):
            next_out = []
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

    subtract_cases = [
        ([], [(1.0, 2.0)]),
        ([(0.0, 10.0)], []),
        ([(0.0, 10.0)], [(-2.0, -1.0), (2.0, 3.0), (5.0, 7.0), (11.0, 12.0)]),
        ([(0.0, 10.0)], [(2.0, 5.0), (4.0, 8.0)]),
        ([(0.0, 1.0)], [(-1.0, 0.5), (0.5 + 0.5e-9, 2.0)]),
        ([(0.0, 1.0)], [(0.1e-9, 2.0)]),
        ([(0.0, 1.0)], [(-1.0, 0.0), (0.5e-9, 2.0)]),
        ([(0.0, 3.0), (5.0, 10.0)], [(1.0, 2.0), (6.0, 9.0)]),
    ]
    for intervals, cuts in subtract_cases:
        expected = subtract_reference(intervals, cuts)
        assert plugin.subtract_intervals(intervals, cuts) == expected
        assert plugin.subtract_intervals(intervals, cuts, True) == expected
        assert plugin.subtract_intervals(intervals, sorted(cuts), True, True) == expected

    pts = [(-2.0, -1.5), (2.0, -1.5), (2.0, 1.5), (-2.0, 1.5), (-2.0, -1.5)]
    segs = []
    for i in range(4):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        segs.append(plugin.Segment(
            x0=x0, y0=y0, z0=0.2,
            x1=x1, y1=y1, z1=0.2,
            de=1.0, length=math.hypot(x1 - x0, y1 - y0), line_index=i,
        ))
    loop = plugin.Loop(layer_index=0, z=0.2, segments=segs, end_insert_index=0)
    loop.build_cum()
    dense = plugin.subdivide_loop(loop, 0.1)

    for px, py in [(-2.5, 0.3), (0.25, 0.1), (1.9, 1.7), (0.0, -1.5)]:
        expected_x, expected_y, expected_d2 = plugin._nearest_in_segments(px, py, dense.segments)
        actual_x, actual_y = plugin.nearest_on_loop((px, py), dense)
        assert (actual_x, actual_y) == (expected_x, expected_y)
        indexed_x, indexed_y, indexed_i, indexed_d2 = plugin.nearest_on_loop_with_index((px, py), dense)
        assert (indexed_x, indexed_y, indexed_d2) == (expected_x, expected_y, expected_d2)
        brute_i = min(
            range(len(dense.segments)),
            key=lambda i: plugin._nearest_in_segments(px, py, dense.segments, [i])[2],
        )
        assert indexed_i == brute_i

    def brute(y: float) -> list[tuple[float, float]]:
        xs = []
        for seg in dense.segments:
            if (seg.y0 > y) == (seg.y1 > y):
                continue
            t = (y - seg.y0) / (seg.y1 - seg.y0)
            xs.append(seg.x0 + (seg.x1 - seg.x0) * t)
        xs.sort()
        return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2) if xs[i + 1] - xs[i] > 1e-9]

    for y in [-1.49, -0.75, 0.0, 1.49, 1.6]:
        assert plugin.scanline_intervals(dense, y) == brute(y), f"scanline index mismatch at y={y}"

    def rect(x0: float, y0: float, x1: float, y1: float, layer: int) -> plugin.Loop:
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        rect_segments = []
        for i in range(4):
            ax, ay = corners[i]
            bx, by = corners[i + 1]
            rect_segments.append(plugin.Segment(
                x0=ax, y0=ay, z0=0.2,
                x1=bx, y1=by, z1=0.2,
                de=1.0, length=math.hypot(bx - ax, by - ay), line_index=i,
            ))
        result = plugin.Loop(layer_index=layer, z=0.2, segments=rect_segments, end_insert_index=0)
        result.build_cum()
        return result

    layered_loops = [rect(-3.0, -2.0, 3.0, 2.0, 0), rect(-1.0, -1.0, 1.0, 1.0, 0),
                     rect(4.0, -0.5, 5.0, 0.5, 1)]
    layered_index = plugin.LayeredMaterialIndex(layered_loops)
    for y in [-1.9, -0.75, 0.0, 0.75, 1.9, 2.1]:
        expected = plugin.scanline_layered_material_intervals(layered_loops, y)
        assert layered_index.intervals(y) == expected, f"layered index mismatch at y={y}"
        assert layered_index.intervals(y) == expected, f"layered index cache mismatch at y={y}"

    stepped_points = [
        (0.0, 0.0), (4.0, 0.0), (4.0, 0.2), (3.0, 0.2),
        (3.0, 0.4), (4.0, 0.4), (4.0, 1.0), (0.0, 1.0), (0.0, 0.0),
    ]
    stepped_segments = []
    for i, ((ax, ay), (bx, by)) in enumerate(zip(stepped_points, stepped_points[1:])):
        stepped_segments.append(plugin.Segment(
            x0=ax, y0=ay, z0=0.2,
            x1=bx, y1=by, z1=0.2,
            de=1.0, length=math.hypot(bx - ax, by - ay), line_index=i,
        ))
    stepped = plugin.Loop(layer_index=0, z=0.2, segments=stepped_segments, end_insert_index=0)
    stepped.build_cum()
    stepped_index = plugin.LayeredMaterialIndex([stepped, rect(5.0, 0.0, 6.0, 1.0, 0)])
    for y in [0.1, 0.3, 0.45]:
        expected = plugin.scanline_layered_material_intervals(
            [stepped, rect(5.0, 0.0, 6.0, 1.0, 0)], y
        )
        assert stepped_index.intervals(y) == expected, f"stepped material index mismatch at y={y}"


def check_audit_material_intervals_semantics() -> None:
    def rect(x0: float, y0: float, x1: float, y1: float, layer: int = 0) -> audit_coverage.Loop:
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        segs = []
        for i in range(4):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            segs.append(audit_coverage.Seg(ax, ay, bx, by, 0.2, 1.0, 1200.0, "wall"))
        return audit_coverage.Loop(layer=layer, z=0.2, segs=segs, area=audit_coverage.loop_area(segs))

    outer = rect(0.0, 0.0, 10.0, 10.0)
    hole = rect(3.0, 3.0, 7.0, 7.0)
    island = rect(12.0, 4.0, 14.0, 6.0)
    intervals = audit_coverage.material_intervals([outer, hole, island], 5.0)
    expected = [(0.0, 3.0), (7.0, 10.0), (12.0, 14.0)]
    assert intervals == expected, f"audit material intervals lost hole/island semantics: {intervals}"
    assert audit_coverage.material_intervals([outer, hole, island], 5.0) == expected


def check_audit_overhang_wall_toggle() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-audit-overhang-"))
    try:
        path = temp_dir / "overhang.gcode"
        path.write_text(
            """M82
G90
G92 E0
;LAYER_CHANGE
;TYPE:Overhang wall
G1 X0 Y0 Z0.2 F1800
G1 X10 Y0 E1 F1200
G1 X10 Y10 E2 F1200
G1 X0 Y10 E3 F1200
G1 X0 Y0 E4 F1200
""",
            encoding="utf-8",
        )
        default_parsed = audit_coverage.parse_file(str(path))
        overhang_parsed = audit_coverage.parse_file(str(path), include_overhang_starts=True)
        assert not default_parsed.loops, "overhang wall should not start a wall loop by default"
        assert len(overhang_parsed.loops[0]) == 1, "include_overhang_starts should parse the overhang loop"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_audit_generated_safety_semantics() -> None:
    def rect_lines(x0: float, y0: float, x1: float, y1: float, e: float) -> tuple[list[str], float]:
        lines = [f"G1 X{x0:g} Y{y0:g} Z0.2 F1800"]
        for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0)):
            e += 1.0
            lines.append(f"G1 X{x:g} Y{y:g} E{e:g} F1200")
        return lines, e

    e = 0.0
    lines = ["M82", "G90", "G92 E0", ";LAYER_CHANGE", ";TYPE:Outer wall"]
    more, e = rect_lines(0.0, 0.0, 10.0, 10.0, e)
    lines.extend(more)
    lines.append(";TYPE:Outer wall")
    more, e = rect_lines(3.0, 3.0, 7.0, 7.0, e)
    lines.extend(more)
    original = "\n".join(lines) + "\n"
    processed = original + (
        "; BOWP spiral start x3\n"
        "G1 X12 Y5 Z0.2 E9 F1200\n"
        "G1 X12.1 Y5 Z0.2 E9.01 F1200\n"
        "G1 X5 Y5 Z0.2 E9.02 F1200\n"
        "G1 X5.1 Y5 Z0.2 E9.03 F1200\n"
        "; BOWP spiral end\n"
    )
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-generated-safety-"))
    try:
        original_path = temp_dir / "original.gcode"
        processed_path = temp_dir / "processed.gcode"
        original_path.write_text(original, encoding="utf-8")
        processed_path.write_text(processed, encoding="utf-8")
        orig = audit_coverage.parse_file(str(original_path))
        proc = audit_coverage.parse_file(str(processed_path))
        safety = audit_coverage.audit_generated_safety(orig, proc, tol=0.1, sample_step=0.8)
        assert safety.external_bad > 0, "generated path outside the model was not reported"
        assert safety.internal_bad > 0, "generated path inside a hole was not reported"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_audit_unneeded_ironing_semantics() -> None:
    def rect_lines(
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        z: float,
        e: float,
    ) -> tuple[list[str], float]:
        lines = [f"G1 X{x0:g} Y{y0:g} Z{z:g} F1800"]
        for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0)):
            e += 1.0
            lines.append(f"G1 X{x:g} Y{y:g} E{e:g} F1200")
        return lines, e

    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-unneeded-ironing-"))
    try:
        e = 0.0
        original_lines = ["M82", "G90", "G92 E0", ";LAYER_CHANGE", ";TYPE:Outer wall"]
        more, e = rect_lines(0.0, 0.0, 10.0, 10.0, 0.2, e)
        original_lines.extend(more)
        original_lines.extend([";LAYER_CHANGE", ";TYPE:Outer wall"])
        more, e = rect_lines(0.0, 0.0, 10.0, 10.0, 0.4, e)
        original_lines.extend(more)
        original = "\n".join(original_lines) + "\n"

        processed_lines = ["M82", "G90", "G92 E0", ";LAYER_CHANGE", ";TYPE:Outer wall"]
        e = 0.0
        more, e = rect_lines(0.0, 0.0, 10.0, 10.0, 0.2, e)
        processed_lines.extend(more)
        processed_lines.extend([
            "; BOWP script ironing start",
            "G1 X2 Y5 Z0.2 F1800",
            f"G1 X8 Y5 E{e + 0.1:g} F1200",
            "; BOWP script ironing end",
            ";LAYER_CHANGE",
            ";TYPE:Outer wall",
        ])
        e += 0.1
        more, e = rect_lines(0.0, 0.0, 10.0, 10.0, 0.4, e)
        processed_lines.extend(more)
        processed = "\n".join(processed_lines) + "\n"

        original_path = temp_dir / "original.gcode"
        processed_path = temp_dir / "processed.gcode"
        original_path.write_text(original, encoding="utf-8")
        processed_path.write_text(processed, encoding="utf-8")
        orig = audit_coverage.parse_file(str(original_path))
        proc = audit_coverage.parse_file(str(processed_path))
        safety = audit_coverage.audit_generated_safety(orig, proc, tol=0.1, sample_step=0.8)
        assert safety.unneeded_ironing_bad > 0, "covered interior script ironing was not reported"

        top_only_path = temp_dir / "top_only.gcode"
        top_only_path.write_text("\n".join(original_lines[:10]) + "\n", encoding="utf-8")
        top_proc_path = temp_dir / "top_proc.gcode"
        top_proc_path.write_text("\n".join(processed_lines[:14]) + "\n", encoding="utf-8")
        top_safety = audit_coverage.audit_generated_safety(
            audit_coverage.parse_file(str(top_only_path)),
            audit_coverage.parse_file(str(top_proc_path)),
            tol=0.1,
            sample_step=0.8,
        )
        assert top_safety.unneeded_ironing_bad == 0, "true top-surface script ironing was misreported"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_audit_pass_count_clusters_segments() -> None:
    segs = [
        audit_coverage.Seg(0.0, 0.0, 0.2, 0.0, 0.2, 0.01, 1200.0, "bowp"),
        audit_coverage.Seg(0.2, 0.0, 0.4, 0.0, 0.2, 0.01, 1200.0, "bowp"),
        audit_coverage.Seg(0.4, 0.0, 0.6, 0.0, 0.2, 0.01, 1200.0, "bowp"),
        audit_coverage.Seg(0.0, 1.0, 0.6, 1.0, 0.2, 0.01, 1200.0, "bowp"),
    ]
    idx = audit_coverage.SegIndex(segs)
    assert idx.count_within(0.3, 0.0, 0.15) >= 2, "segment count setup no longer exercises dense segments"
    assert idx.pass_count_within(0.3, 0.0, 0.15) == 1, "continuous dense segments should count as one pass"
    assert idx.pass_count_within(0.3, 0.5, 0.55) == 2, "separate nearby polylines should count as two passes"


def check_audit_transition_continuity() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-transition-"))
    try:
        path = temp_dir / "transition.gcode"
        path.write_text(
            """M82
G92 E0
;LAYER_CHANGE
; BOWP secondary pass start
G1 X0 Y0 Z0.2 F1200
G1 X1 Y0 E0.01000 F1200
; BOWP secondary pass end
; BOWP script ironing start
G1 X2 Y0 E0.02000 F2400
; BOWP script ironing end
""",
            encoding="utf-8",
        )
        parsed = audit_coverage.parse_file(str(path))
        assert parsed.continuity.transition_pairs == 1, (
            f"expected one generated transition pair, got {parsed.continuity.transition_pairs}"
        )
        assert_close(parsed.continuity.max_transition_feed_ratio, 2.0, 1e-9, "transition feed ratio")
        assert_close(parsed.continuity.max_transition_flow_ratio, 1.0, 1e-9, "transition flow ratio")

        path.write_text(
            """M82
G92 E0
;LAYER_CHANGE
; BOWP secondary pass start
G1 X0 Y0 Z0.2 F1200
G1 X1 Y0 E0.01000 F1200
; BOWP secondary pass end
; BOWP script ironing start
G0 X1.2 Y0 F9000
G1 X2.2 Y0 E0.02000 F6000
; BOWP script ironing end
""",
            encoding="utf-8",
        )
        separated = audit_coverage.parse_file(str(path))
        assert separated.continuity.transition_pairs == 0, (
            "travel-separated generated moves should not count as a continuous transition"
        )
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_audit_arc_expansion() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-audit-arc-"))
    try:
        path = temp_dir / "arc.gcode"
        path.write_text(
            """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X10 Y0 Z0.2 F1800
G2 X-10 Y0 I-10 J0 E3.14159 F1200
G2 X10 Y0 I10 J0 E6.28318
""",
            encoding="utf-8",
        )
        parsed = audit_coverage.parse_file(str(path))
        loops = parsed.loops[0]
        assert len(loops) == 1, f"expected one expanded arc loop, got {len(loops)}"
        loop = loops[0]
        assert len(loop.segs) > 40, f"arc loop was not expanded enough: {len(loop.segs)}"
        assert abs(abs(loop.area) - math.pi * 100.0) < 8.0, f"arc area wrong: {loop.area}"
        intervals = audit_coverage.material_intervals(loops, 1.0)
        expected = math.sqrt(100.0 - 1.0)
        assert len(intervals) == 1, f"unexpected arc intervals: {intervals}"
        assert_close(intervals[0][0], -expected, 0.2, "arc left crossing")
        assert_close(intervals[0][1], expected, 0.2, "arc right crossing")
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_processor_arc_travel_split() -> None:
    gcode = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X10 Y0 E1 F1200
G1 X10 Y10 E2
G1 X0 Y10 E3
G1 X0 Y0 E4
G2 X30 Y0 I15 J0 F9000
G1 X40 Y0 E5 F1200
G1 X40 Y10 E6
G1 X30 Y10 E7
G1 X30 Y0 E8
"""
    lines = [plugin.parse_line(raw) for raw in gcode.splitlines(keepends=True)]
    infos, _, _, _ = plugin.annotate(lines)
    loops, skipped = plugin.build_loops(infos, sample_step=0.5)
    assert skipped == 0, f"arc travel split should not taint loops: skipped={skipped}"
    assert len(loops) == 2, f"arc travel should split islands, got {len(loops)} loops"
    centers = sorted(round(sum(seg.x0 for seg in loop.segments) / len(loop.segments), 1) for loop in loops)
    assert centers == [5.0, 35.0], f"unexpected loop centers after arc travel split: {centers}"


def check_seam_leadin_closed_loop() -> None:
    gcode = """M82
G90
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.20 F1800
G1 X10 Y0 E1 F1200
G1 X10 Y10 E2
G1 X0 Y10 E3
G1 X0 Y0 E4
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0.20 Y0 Z0.40 F1800
G1 X0 Y0 E4.02 F1200
G1 X10 Y0 E5
G1 X10 Y10 E6
G1 X0 Y10 E7
G1 X0 Y0 E8
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.60 F1800
G1 X10 Y0 E9 F1200
G1 X10 Y10 E10
G1 X0 Y10 E11
G1 X0 Y0 E12
"""
    cfg = dict(
        plugin.DEFAULT_CONFIG,
        progress_enabled=False,
        script_ironing_enabled=False,
        secondary_pass_enabled=False,
        purge_retract_mm=0.0,
        seam_mode="spiral",
        seam_superres_x=3,
        max_overhang_deg=-90.0,
        spiral_flatten_enabled=False,
    )
    lines = [plugin.parse_line(raw) for raw in gcode.splitlines(keepends=True)]
    infos, _, _, _ = plugin.annotate(lines)
    loops, skipped = plugin.build_loops(infos)
    assert skipped == 0, f"lead-in loop should not be arc-tainted: {skipped}"
    assert len(loops) == 3, f"expected 3 loops, got {len(loops)}"
    target = loops[1]
    assert plugin.loop_is_closed(target), "short seam lead-in should still be a closed loop"
    assert len(plugin.closed_loop_geometry(target).segments) == len(target.segments) - 1
    prev_map, next_map = plugin.match_neighbor_loops(loops)
    heights = plugin.layer_heights(loops, prev_map)
    mode, reason = plugin.decide_mode(
        target, cfg, heights.get(id(target), 0.0),
        prev_map.get(id(target)), next_map.get(id(target)),
        float(cfg.get("sample_step_mm", 1.0)),
    )
    assert (mode, reason) == ("spiral", "requested"), f"lead-in loop demoted: {mode=} {reason=}"

    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-leadin-"))
    try:
        path = temp_dir / "lead_in.gcode"
        path.write_text(gcode, encoding="utf-8")
        plugin.run_processor(str(path), cfg)
        out = path.read_text(encoding="utf-8")
        assert out.count("; BOWP spiral start x3") >= 2, "lead-in layer was not spiral-processed"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_nested_hole_boundary_not_generated() -> None:
    gcode = """M82
G90
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.20 F1800
G1 X30 Y0 E1 F1200
G1 X30 Y30 E2
G1 X0 Y30 E3
G1 X0 Y0 E4
;TYPE:Inner wall
G1 F7200
;TYPE:Outer wall
G1 X10 Y10 F1800
G1 X10 Y20 E5 F1200
G1 X20 Y20 E6
G1 X20 Y10 E7
G1 X10 Y10 E8
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.40 F1800
G1 X30 Y0 E9 F1200
G1 X30 Y30 E10
G1 X0 Y30 E11
G1 X0 Y0 E12
;TYPE:Inner wall
G1 F7200
;TYPE:Outer wall
G1 X10 Y10 F1800
G1 X10 Y20 E13 F1200
G1 X20 Y20 E14
G1 X20 Y10 E15
G1 X10 Y10 E16
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.60 F1800
G1 X30 Y0 E17 F1200
G1 X30 Y30 E18
G1 X0 Y30 E19
G1 X0 Y0 E20
;TYPE:Inner wall
G1 F7200
;TYPE:Outer wall
G1 X10 Y10 F1800
G1 X10 Y20 E21 F1200
G1 X20 Y20 E22
G1 X20 Y10 E23
G1 X10 Y10 E24
"""
    cfg = dict(
        plugin.DEFAULT_CONFIG,
        progress_enabled=False,
        script_ironing_enabled=False,
        secondary_pass_enabled=False,
        purge_retract_mm=0.0,
        seam_mode="spiral",
        seam_superres_x=3,
        max_overhang_deg=-90.0,
    )
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-hole-boundary-"))
    try:
        path = temp_dir / "hole_boundary.gcode"
        path.write_text(gcode, encoding="utf-8")
        seams, _ = plugin.run_processor(str(path), cfg)
        out = path.read_text(encoding="utf-8")
        assert seams == 3, f"nested hole boundaries should not be BOWP-generated: {seams}"
        assert out.count("; BOWP spiral start x3") == 2, "only non-first-layer outer boundaries should spiral"
        in_bowp = False
        for raw in out.splitlines():
            if raw.startswith("; BOWP spiral start"):
                in_bowp = True
                continue
            if raw.startswith("; BOWP spiral end"):
                in_bowp = False
                continue
            if not in_bowp:
                continue
            line = plugin.parse_line(raw + "\n")
            if line.command in ("G0", "G1") and "X" in line.args and "Y" in line.args:
                assert not (9.5 <= line.args["X"] <= 20.5 and 9.5 <= line.args["Y"] <= 20.5), (
                    f"BOWP generated inside the nested hole boundary: {raw}"
                )
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_single_wall_mask_toggle() -> None:
    def add_square(lines: list[str], x0: float, y0: float, x1: float, y1: float, z: float, e: float) -> float:
        lines.append(f"G1 X{x0:g} Y{y0:g} Z{z:.2f} F1800")
        for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0)):
            e += 1.0
            lines.append(f"G1 X{x:g} Y{y:g} E{e:g} F1200")
        return e

    lines = ["M82", "G90", "G92 E0"]
    e = 0.0
    for layer, z in enumerate((0.20, 0.40, 0.60)):
        lines.append(";LAYER_CHANGE")
        lines.append(";TYPE:Outer wall")
        e = add_square(lines, 0.0, 0.0, 20.0, 20.0, z, e)
        lines.append(";TYPE:Inner wall")
        e = add_square(lines, 0.6, 0.6, 19.4, 19.4, z, e)
        lines.append(";TYPE:Outer wall")
        e = add_square(lines, 40.0, 0.0, 50.0, 10.0, z, e)
    gcode = "\n".join(lines) + "\n"
    cfg = dict(
        plugin.DEFAULT_CONFIG,
        progress_enabled=False,
        script_ironing_enabled=False,
        secondary_pass_enabled=False,
        purge_retract_mm=0.0,
        seam_mode="spiral",
        seam_superres_x=3,
        max_overhang_deg=-90.0,
    )
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-single-wall-mask-"))
    try:
        default_path = temp_dir / "default_off.gcode"
        default_path.write_text(gcode, encoding="utf-8")
        plugin.run_processor(str(default_path), cfg)
        default_out = default_path.read_text(encoding="utf-8")
        assert default_out.count("; BOWP spiral start x3") == 4, "default-off mask changed generated loops"

        masked_path = temp_dir / "mask_on.gcode"
        masked_path.write_text(gcode, encoding="utf-8")
        plugin.run_processor(str(masked_path), dict(cfg, single_wall_mask_enabled=True))
        masked_out = masked_path.read_text(encoding="utf-8")
        assert masked_out.count("; BOWP spiral start x3") == 2, "single-wall loop was not masked"
        in_bowp = False
        for raw in masked_out.splitlines():
            if raw.startswith("; BOWP spiral start"):
                in_bowp = True
                continue
            if raw.startswith("; BOWP spiral end"):
                in_bowp = False
                continue
            if not in_bowp:
                continue
            line = plugin.parse_line(raw + "\n")
            if line.command in ("G0", "G1") and "X" in line.args and "Y" in line.args:
                assert line.args["X"] < 30.0, f"masked single-wall region received generated path: {raw}"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_relative_coordinate_mode() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-relative-xy-"))
    try:
        path = temp_dir / "relative_xy.gcode"
        gcode = """G91
M83
;LAYER_CHANGE
;TYPE:Outer wall
G1 Z0.2 F1800
G1 X10 Y0 E1 F1200
G1 X0 Y10 E1
G1 X-10 Y0 E1
G1 X0 Y-10 E1
;LAYER_CHANGE
;TYPE:Outer wall
G1 Z0.2 F1800
G1 X10 Y0 E1 F1200
G1 X0 Y10 E1
G1 X-10 Y0 E1
G1 X0 Y-10 E1
"""
        path.write_text(gcode, encoding="utf-8")
        lines = [plugin.parse_line(raw) for raw in gcode.splitlines(keepends=True)]
        infos, _, _, _ = plugin.annotate(lines)
        loops, _ = plugin.build_loops(infos)
        assert len(loops) == 2, f"relative XY loops not parsed: {len(loops)}"
        assert all(abs(abs(plugin.loop_signed_area(loop)) - 100.0) < 1e-6 for loop in loops)

        parsed = audit_coverage.parse_file(str(path))
        audit_loops = [lp for layer in sorted(parsed.loops) for lp in parsed.loops[layer]]
        assert len(audit_loops) == 2, f"audit relative XY loops not parsed: {len(audit_loops)}"
        assert all(abs(abs(lp.area) - 100.0) < 1e-6 for lp in audit_loops)

        cfg = dict(plugin.DEFAULT_CONFIG, progress_enabled=False, script_ironing_enabled=False, purge_retract_mm=0.0)
        seams, _ = plugin.run_processor(str(path), cfg)
        assert seams == 2, f"relative XY file not processed: {seams}"
        replay = simulate(path)
        assert replay["max_xy_jump_extruding"] <= 20.1, (
            f"relative coordinate render produced a large jump: {replay['max_xy_jump_extruding']}"
        )
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_g92_coordinate_reset() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-g92-xyz-"))
    try:
        path = temp_dir / "g92_xyz.gcode"
        gcode = """G90
M83
G1 X50 Y50 Z0.2 F1800
G92 X0 Y0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X10 Y0 E1 F1200
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.4 F1800
G1 X10 Y0 E1 F1200
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
"""
        path.write_text(gcode, encoding="utf-8")
        lines = [plugin.parse_line(raw) for raw in gcode.splitlines(keepends=True)]
        infos, _, _, _ = plugin.annotate(lines)
        loops, _ = plugin.build_loops(infos)
        assert len(loops) == 2, f"G92 XYZ loops not parsed: {len(loops)}"
        assert all(abs(abs(plugin.loop_signed_area(loop)) - 100.0) < 1e-6 for loop in loops)

        parsed = audit_coverage.parse_file(str(path))
        audit_loops = [lp for layer in sorted(parsed.loops) for lp in parsed.loops[layer]]
        assert len(audit_loops) == 2, f"audit G92 XYZ loops not parsed: {len(audit_loops)}"
        assert all(abs(abs(lp.area) - 100.0) < 1e-6 for lp in audit_loops)

        cfg = dict(plugin.DEFAULT_CONFIG, progress_enabled=False, script_ironing_enabled=False, purge_retract_mm=0.0)
        seams, _ = plugin.run_processor(str(path), cfg)
        assert seams == 2, f"G92 XYZ file not processed: {seams}"
        replay = simulate(path)
        assert replay["max_xy_jump_extruding"] <= 20.1, (
            f"G92 coordinate render produced a large jump: {replay['max_xy_jump_extruding']}"
        )
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_legacy_bowp_marker_skip() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-legacy-marker-"))
    try:
        path = temp_dir / "legacy_step_only.gcode"
        prefix = "".join(f"; filler {i}\n" for i in range(32))
        text = (
            "M82\nG92 E0\n"
            + prefix
            + "; BOWP step ironing start\n"
            + "G1 X0 Y0 Z0.2 E0.01 F1200\n"
            + "; BOWP step ironing end\n"
            + ";LAYER_CHANGE\n;TYPE:Outer wall\n"
            + "G1 X0 Y0 Z0.2 F1800\nG1 X1 Y0 E1\n"
        )
        path.write_text(text, encoding="utf-8")
        seams, stats = plugin.run_processor(str(path), dict(plugin.DEFAULT_CONFIG, progress_enabled=False))
        assert seams == 0, f"legacy step-ironing marker should be skipped, got {seams}"
        assert stats.final_time_s == 0.0, "processed skip should return empty stats"
        assert path.read_text(encoding="utf-8") == text, "legacy processed file was modified"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_atomic_output_temp_retry(base: Path) -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-temp-retry-"))
    original_named_temporary_file = plugin.tempfile.NamedTemporaryFile
    calls = {"count": 0}

    def flaky_named_temporary_file(*args, **kwargs):
        calls["count"] += 1
        if calls["count"] == 1:
            raise FileNotFoundError("simulated transient temp output failure")
        return original_named_temporary_file(*args, **kwargs)

    try:
        path = temp_dir / "retry.gcode"
        shutil.copyfile(base / "sample_input.gcode", path)
        plugin.tempfile.NamedTemporaryFile = flaky_named_temporary_file
        seams, _ = plugin.run_processor(str(path), dict(plugin.DEFAULT_CONFIG, progress_enabled=False))
        assert seams > 0, "processor did not retry after transient temp output failure"
        assert calls["count"] >= 2, "temporary output creation was not retried"
        leftovers = list(temp_dir.glob(".bowp-*.gcode"))
        assert not leftovers, f"temporary output files were left behind: {leftovers}"
    finally:
        plugin.tempfile.NamedTemporaryFile = original_named_temporary_file
        shutil.rmtree(temp_dir, ignore_errors=True)


def check_audit_layer_section_reset() -> None:
    temp_dir = Path(tempfile.mkdtemp(prefix="bowp-audit-reset-"))
    try:
        path = temp_dir / "section_reset.gcode"
        path.write_text(
            """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X10 Y0 E1
G1 X10 Y10 E2
G1 X0 Y10 E3
G1 X0 Y0 E4
;LAYER_CHANGE
G1 X30 Y30 Z0.4 F1800
G1 X40 Y30 E5
G1 X40 Y40 E6
G1 X30 Y40 E7
G1 X30 Y30 E8
""",
            encoding="utf-8",
        )
        parsed = audit_coverage.parse_file(str(path))
        assert sorted(parsed.wall) == [0], f"audit section leaked across layer: {sorted(parsed.wall)}"
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def run() -> None:
    base = Path(__file__).resolve().parent
    plugin_text = (base / "better_outer_wall_processing.py").read_text(encoding="utf-8")
    manual_text = (base / "使用说明书.md").read_text(encoding="utf-8")
    html_text = plugin.build_html(plugin.DEFAULT_CONFIG)

    for value in [
        "更好的外墙处理",
        "BOWP secondary pass start",
        "BOWP spiral start",
        "seam_superres_x",
        "seam_mode",
        "spiral_flatten_enabled",
        "spiral_xy_interp_enabled",
        "travel_speed_mm_s",
        "script_ironing_enabled",
        "path_width_mm",
    ]:
        assert_contains(plugin_text, value)
    assert "step_ironing" not in plugin_text, "step ironing code should be removed"

    for value in [
        "更好的外墙处理",
        "亚层螺旋",
        "超分倍数",
        "空转压平",
        "亚层级精度优化",
        "参数说明",
        "回退方式",
        "脚本熨烫",
        "替代切片器熨烫",
    ]:
        assert_contains(manual_text, value)

    field_keys = [field["key"] for field in plugin.CONFIG_FIELDS]
    assert len(field_keys) == len(set(field_keys)), "CONFIG_FIELDS contains duplicate keys"
    assert set(field_keys) == set(plugin.DEFAULT_CONFIG), "CONFIG_FIELDS and DEFAULT_CONFIG keys diverged"
    for field in plugin.CONFIG_FIELDS:
        key = field["key"]
        kind = field["kind"]
        assert kind in {"bool", "int", "float", "enum"}, f"unexpected field kind for {key}: {kind}"
        if kind == "enum":
            assert field.get("options"), f"enum field missing options: {key}"
        assert_contains(html_text, f'id="{key}"')

    check_config_roundtrip()
    check_audit_coverage_script(base)
    check_validate_gcode_script(base)
    check_validate_recursive_discovery()
    check_scanline_index_consistency()
    check_audit_material_intervals_semantics()
    check_audit_overhang_wall_toggle()
    check_audit_generated_safety_semantics()
    check_audit_unneeded_ironing_semantics()
    check_audit_pass_count_clusters_segments()
    check_audit_transition_continuity()
    check_audit_arc_expansion()
    check_processor_arc_travel_split()
    check_seam_leadin_closed_loop()
    check_nested_hole_boundary_not_generated()
    check_single_wall_mask_toggle()
    check_relative_coordinate_mode()
    check_g92_coordinate_reset()
    check_legacy_bowp_marker_skip()
    check_atomic_output_temp_retry(base)
    check_audit_layer_section_reset()

    cfg = dict(plugin.DEFAULT_CONFIG)
    cfg["progress_enabled"] = False
    # 既有守恒/几何用例只验证外墙处理本身；脚本熨烫另有专门用例。
    cfg["script_ironing_enabled"] = False

    # 1. 螺旋模式（绝对挤出 + 回抽 + G92）
    check_processed(base / "sample_input.gcode", cfg)
    check_spiral_geometry(base / "sample_input.gcode", cfg)

    # 2. 螺旋 x=3
    cfg3 = dict(cfg, seam_superres_x=3)
    check_processed(base / "sample_input.gcode", cfg3)
    check_spiral_geometry(base / "sample_input.gcode", cfg3)

    # 3. 斜接缝模式
    check_processed(base / "sample_input.gcode", dict(cfg, seam_mode="scarf"))

    # 4. 平面模式
    check_processed(base / "sample_input.gcode", dict(cfg, seam_mode="flat"))

    # 5. 相对挤出（M83）
    check_processed(base / "sample_input_relative.gcode", cfg)

    # 6. 关闭二次整形 / 关闭压平
    check_processed(base / "sample_input.gcode", dict(cfg, secondary_pass_enabled=False))
    check_processed(base / "sample_input.gcode", dict(cfg, spiral_flatten_enabled=False))

    # 7. XY 插值：第二层（19.8 方形）螺旋下半程应向第一层（20 方形）外扩；
    #    关闭插值时所有子圈 XY 恒为 19.8 轮廓
    def spiral_max_x(interp: bool) -> float:
        d = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
        try:
            w = d / "interp.gcode"
            shutil.copyfile(base / "sample_input.gcode", w)
            plugin.run_processor(str(w), dict(cfg, spiral_xy_interp_enabled=interp))
            lines = w.read_text(encoding="utf-8").splitlines()
            # 用第二层（min）：向前一层（20 方形）外扩
            begin = min(i for i, l in enumerate(lines) if l.startswith("; BOWP spiral start"))
            end = next(i for i in range(begin, len(lines)) if lines[i].startswith("; BOWP spiral top fill"))
            mx = 0.0
            for raw in lines[begin:end]:
                line = plugin.parse_line(raw + "\n")
                if line.command == "G1" and "X" in line.args:
                    mx = max(mx, line.args["X"])
            return mx
        finally:
            shutil.rmtree(d, ignore_errors=True)

    mx_on = spiral_max_x(True)
    mx_off = spiral_max_x(False)
    assert mx_off <= 19.8 + 1e-6, f"no-interp spiral leaked outward: {mx_off}"
    assert mx_on > 19.83, f"interp did not shift toward previous layer: {mx_on}"

    # 8. 细分生效：sample_step 0.5 的螺旋行数应明显多于 4.0
    def spiral_line_count(step: float) -> int:
        d = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
        try:
            w = d / "step.gcode"
            shutil.copyfile(base / "sample_input.gcode", w)
            plugin.run_processor(str(w), dict(cfg, sample_step_mm=step))
            lines = w.read_text(encoding="utf-8").splitlines()
            begin = max(i for i, l in enumerate(lines) if l.startswith("; BOWP spiral start"))
            end = next(i for i in range(begin, len(lines)) if lines[i].startswith("; BOWP spiral end"))
            return sum(1 for l in lines[begin:end] if l.startswith("G1 "))
        finally:
            shutil.rmtree(d, ignore_errors=True)

    fine, coarse = spiral_line_count(0.5), spiral_line_count(4.0)
    assert fine > coarse * 2, f"subdivision has no effect: fine={fine}, coarse={coarse}"

    # 9. 圆弧展开：含 G2 的外墙应被正常处理（不再跳过），挤出守恒
    arc_gcode = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X10 Y0 Z0.2 F1800
G2 X-10 Y0 I-10 J0 E3.14159 F1200
G2 X10 Y0 I10 J0 E6.28318
;TYPE:Inner wall
G1 X0 Y0 E6.5
;LAYER_CHANGE
;TYPE:Outer wall
G1 X10 Y0 Z0.4 F1800
G2 X-10 Y0 I-10 J0 E9.42477 F1200
G2 X10 Y0 I10 J0 E12.56636
;LAYER_CHANGE
;TYPE:Outer wall
G1 X10 Y0 Z0.6 F1800
G2 X-10 Y0 I-10 J0 E15.70795 F1200
G2 X10 Y0 I10 J0 E18.84954
"""
    d = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    try:
        w = d / "arc.gcode"
        w_orig = d / "arc_orig.gcode"
        w.write_text(arc_gcode, encoding="utf-8")
        w_orig.write_text(arc_gcode, encoding="utf-8")
        seams, stats = plugin.run_processor(str(w), cfg)
        assert seams == 3, f"arc loops not processed: {seams}"
        assert stats.arc_loops_skipped == 0, "arc loop wrongly skipped"
        text = w.read_text(encoding="utf-8")
        assert "BOWP spiral start" in text, "arc loop did not get spiral"
        replay = simulate(w)
        exp = expected_net_extrusion(w_orig, cfg)
        assert_close(replay["net_extrusion"], exp, 1e-3, "arc extrusion conservation")
    finally:
        shutil.rmtree(d, ignore_errors=True)

    # 10. 短墙三档：极限以下不处理；极限~min_wall 兜底处理；以上正常
    short_gcode = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X0.2 Y0 E0.01 F1200
G1 X0.2 Y0.2 E0.02
G1 X0 Y0.2 E0.03
G1 X0 Y0 E0.04
;TYPE:Outer wall
G1 X10 Y10 F9000
G1 X10.6 Y10 E0.07 F1200
G1 X10.6 Y10.6 E0.1
G1 X10 Y10.6 E0.13
G1 X10 Y10 E0.16
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.4 F1800
G1 X0.2 Y0 E0.17 F1200
G1 X0.2 Y0.2 E0.18
G1 X0 Y0.2 E0.19
G1 X0 Y0 E0.2
;TYPE:Outer wall
G1 X10 Y10 F9000
G1 X10.6 Y10 E0.23 F1200
G1 X10.6 Y10.6 E0.26
G1 X10 Y10.6 E0.29
G1 X10 Y10 E0.32
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.6 F1800
G1 X0.2 Y0 E0.33 F1200
G1 X0.2 Y0.2 E0.34
G1 X0 Y0.2 E0.35
G1 X0 Y0 E0.36
;TYPE:Outer wall
G1 X10 Y10 F9000
G1 X10.6 Y10 E0.39 F1200
G1 X10.6 Y10.6 E0.42
G1 X10 Y10.6 E0.45
G1 X10 Y10 E0.48
"""
    d = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    try:
        w = d / "short.gcode"
        w.write_text(short_gcode, encoding="utf-8")
        # 0.8mm 周长 < 极限 1.0；2.4mm 周长也低于短墙安全阈值：
        # 两者都只保留原始切片器路径，不额外生成螺旋/二次/排压。
        cfg_short = dict(cfg, secondary_pass_enabled=False)
        seams, _ = plugin.run_processor(str(w), cfg_short)
        text = w.read_text(encoding="utf-8")
        assert seams == 6, f"expected 6 loops counted, got {seams}"
        assert text.count("BOWP spiral start") == 0, (
            f"short safety loop unexpectedly spiraled: {text.count('BOWP spiral start')}"
        )
    finally:
        shutil.rmtree(d, ignore_errors=True)

    # 11. 脚本熨烫：无原生熨烫时默认生成裸露顶面平行填线；
    #     文件自带 ;TYPE:Ironing 时走原生修剪方案，不叠加脚本熨烫。
    top_gcode = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X20 Y0 E1.0 F1200
G1 X20 Y20 E2.0
G1 X0 Y20 E3.0
G1 X0 Y0 E4.0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.4 F1800
G1 X20 Y0 E5.0 F1200
G1 X20 Y20 E6.0
G1 X0 Y20 E7.0
G1 X0 Y0 E8.0
"""
    d = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    try:
        w = d / "script_ironing.gcode"
        w.write_text(top_gcode, encoding="utf-8")
        _, stats = plugin.run_processor(str(w), dict(
            cfg,
            script_ironing_enabled=True,
            secondary_pass_enabled=False,
            purge_retract_mm=0.0,
        ))
        text = w.read_text(encoding="utf-8")
        assert stats.script_ironing_lines > 0, "script ironing did not generate lines"
        assert_contains(text, "BOWP script ironing start")
        w2 = d / "native_ironing.gcode"
        w2.write_text(top_gcode + ";TYPE:Ironing\nG1 X2 Y2 E8.01 F1800\n", encoding="utf-8")
        _, stats2 = plugin.run_processor(str(w2), dict(
            cfg,
            script_ironing_enabled=True,
            secondary_pass_enabled=False,
            purge_retract_mm=0.0,
        ))
        assert stats2.script_ironing_lines == 0, "native ironing file should not get script ironing"
        assert "BOWP script ironing start" not in w2.read_text(encoding="utf-8")

        sample_lines = [
            plugin.parse_line(raw)
            for raw in (base / "sample_input.gcode").read_text(encoding="utf-8").splitlines(keepends=True)
        ]
        sample_infos, *_ = plugin.annotate(sample_lines)
        sample_loops, _ = plugin.build_loops(sample_infos, float(cfg.get("sample_step_mm", 1.0)))
        sample_layers = [lp.layer_index for lp in sample_loops]
        assert sample_layers == [0, 1, 2], f"shrinking sample layer assignment regressed: {sample_layers}"

        w3 = d / "shrinking_top.gcode"
        shutil.copyfile(base / "sample_input.gcode", w3)
        _, stats3 = plugin.run_processor(str(w3), dict(
            cfg,
            script_ironing_enabled=True,
            secondary_pass_enabled=False,
            purge_retract_mm=0.0,
        ))
        assert stats3.script_ironing_lines > 0, "shrinking top surface did not get script ironing"
        assert_contains(w3.read_text(encoding="utf-8"), "BOWP script ironing start")

        split_cover_gcode = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X40 Y0 E1 F1200
G1 X40 Y20 E2
G1 X0 Y20 E3
G1 X0 Y0 E4
;TYPE:Sparse infill
G1 X5 Y5 E4.2 F1800
G1 X35 Y15 E4.8
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.4 F1800
G1 X20 Y0 E5 F1200
G1 X20 Y20 E6
G1 X0 Y20 E7
G1 X0 Y0 E8
;TYPE:Outer wall
G1 X20 Y0 F1800
G1 X40 Y0 E9 F1200
G1 X40 Y20 E10
G1 X20 Y20 E11
G1 X20 Y0 E12
"""
        w4 = d / "split_next_cover.gcode"
        w4.write_text(split_cover_gcode, encoding="utf-8")
        _, stats4 = plugin.run_processor(str(w4), dict(
            cfg,
            script_ironing_enabled=True,
            secondary_pass_enabled=False,
            purge_retract_mm=0.0,
        ))
        text4 = w4.read_text(encoding="utf-8")
        first_layer_out = text4.split(";LAYER_CHANGE", 2)[1]
        assert "BOWP script ironing start" not in first_layer_out, (
            "covered sparse-infill layer should not receive whole-layer script ironing"
        )
        assert stats4.script_ironing_lines > 0, "true top split layer should still receive script ironing"
    finally:
        shutil.rmtree(d, ignore_errors=True)

    # 12. 悬垂降级：第二层向外扩 0.3mm（悬垂角 ~56° > 30°）应降级为 flat
    overhang_gcode = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X20 Y0 E1.0 F1200
G1 X20 Y20 E2.0
G1 X0 Y20 E3.0
G1 X0 Y0 E4.0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X-0.3 Y-0.3 Z0.4 F1800
G1 X20.3 Y-0.3 E5.0 F1200
G1 X20.3 Y20.3 E6.0
G1 X-0.3 Y20.3 E7.0
G1 X-0.3 Y-0.3 E8.0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X-0.3 Y-0.3 Z0.6 F1800
G1 X20.3 Y-0.3 E9.0 F1200
G1 X20.3 Y20.3 E10.0
G1 X-0.3 Y20.3 E11.0
G1 X-0.3 Y-0.3 E12.0
"""
    d = Path(tempfile.mkdtemp(prefix="bowp-selftest-"))
    try:
        w = d / "overhang.gcode"
        w.write_text(overhang_gcode, encoding="utf-8")
        # 默认 -90 关闭检测；显式 -15 验证降级逻辑仍工作
        _, stats = plugin.run_processor(str(w), dict(cfg, max_overhang_deg=-15.0))
        text = w.read_text(encoding="utf-8")
        assert stats.overhang_demoted == 1, f"overhang not demoted: {stats.overhang_demoted}"
        # 关闭检测时应照常螺旋
        w2 = d / "overhang2.gcode"
        w2.write_text(overhang_gcode, encoding="utf-8")
        _, stats2 = plugin.run_processor(str(w2), dict(cfg, max_overhang_deg=0.0))
        assert stats2.overhang_demoted == 0
        assert w2.read_text(encoding="utf-8").count("BOWP spiral start") == 2
        assert "BOWP spiral start" in w2.read_text(encoding="utf-8")
        overhang_only = """M82
G92 E0
;LAYER_CHANGE
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X20 Y0 E1 F1200
G1 X20 Y20 E2
G1 X0 Y20 E3
G1 X0 Y0 E4
;LAYER_CHANGE
;TYPE:Overhang wall
G1 X0 Y0 Z0.4 F1800
G1 X20 Y0 E5 F1200
G1 X20 Y20 E6
G1 X0 Y20 E7
G1 X0 Y0 E8
;LAYER_CHANGE
;TYPE:Overhang wall
G1 X0 Y0 Z0.6 F1800
G1 X20 Y0 E9 F1200
G1 X20 Y20 E10
G1 X0 Y20 E11
G1 X0 Y0 E12
"""
        w3 = d / "overhang_only.gcode"
        w3.write_text(overhang_only, encoding="utf-8")
        seams3, _ = plugin.run_processor(str(w3), dict(
            cfg,
            max_overhang_deg=-90.0,
            script_ironing_enabled=False,
            secondary_pass_enabled=False,
            purge_retract_mm=0.0,
        ))
        assert seams3 == 3, f"standalone overhang-wall loops should be processed when overhang checks are disabled: {seams3}"
    finally:
        shutil.rmtree(d, ignore_errors=True)

    print("selftest_ok")


if __name__ == "__main__":
    run()
