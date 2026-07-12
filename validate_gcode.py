from __future__ import annotations

import argparse
import shlex
import shutil
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import audit_coverage
import better_outer_wall_processing as plugin


MARKERS = (
    "BOWP processed",
    "BOWP spiral",
    "BOWP secondary pass",
    "BOWP script ironing",
    "BOWP step ironing",
    "BOWP scarf overlap",
)


def bowp_marker_counts(path: Path, max_lines: int | None = 20000) -> dict[str, int]:
    counts = {marker: 0 for marker in MARKERS}
    with path.open("r", encoding="utf-8", errors="ignore") as source:
        for index, raw in enumerate(source):
            for marker in MARKERS:
                if marker in raw:
                    counts[marker] += 1
            if max_lines is not None and index >= max_lines:
                break
    return counts


def is_bowp_processed(path: Path) -> bool:
    with path.open("r", encoding="utf-8", errors="ignore") as source:
        return any(any(marker in raw for marker in MARKERS) for raw in source)


def cleanup_temp_dir(path: Path, attempts: int = 5) -> bool:
    for attempt in range(attempts):
        try:
            shutil.rmtree(path)
            return True
        except FileNotFoundError:
            return True
        except OSError:
            if attempt == attempts - 1:
                return False
            time.sleep(0.2 * (attempt + 1))
    return not path.exists()


@dataclass(slots=True)
class ValidationResult:
    status: str
    path: Path
    process_s: float = 0.0
    audit_s: float = 0.0
    wall_bad: int = 0
    surface_bad: int = 0
    top_missing: int = 0
    generated_external_bad: int = 0
    generated_internal_bad: int = 0
    generated_unneeded_ironing_bad: int = 0
    generated_safety_samples: int = 0
    feed_ratio: float = 1.0
    flow_ratio: float = 1.0
    transition_feed_ratio: float = 1.0
    transition_flow_ratio: float = 1.0
    pass_cv: float = 0.0
    script_ironing_lines: int = 0
    input_processed: bool = False
    legacy_step_ironing: bool = False
    current_script_ironing: bool = False
    spiral_markers: int = 0


def discover_files(
    root: Path,
    limit: int | None,
    min_size_mb: float | None,
    max_size_mb: float | None,
    recursive: bool = False,
) -> list[Path]:
    candidates = root.rglob("*") if recursive else root.iterdir()
    files = sorted(
        (p for p in candidates if p.is_file() and p.suffix.lower() in {".gcode", ".gco"}),
        key=lambda p: (p.stat().st_size, str(p).lower()),
    )
    if min_size_mb is not None:
        min_bytes = min_size_mb * 1024 * 1024
        files = [p for p in files if p.stat().st_size >= min_bytes]
    if max_size_mb is not None:
        max_bytes = max_size_mb * 1024 * 1024
        files = [p for p in files if p.stat().st_size <= max_bytes]
    return files[:limit] if limit is not None else files


def processor_args_from_command_text(text: str) -> list[str]:
    tokens = [token.strip('"') for token in shlex.split(text.strip(), posix=False)]
    if tokens and not tokens[0].startswith("-"):
        tokens = tokens[1:]
    return tokens


def build_processor_config(args: argparse.Namespace, work: Path) -> dict:
    cfg = plugin.load_config()
    processor_argv = list(getattr(args, "processor_argv", []))
    if processor_argv:
        cfg, _ = plugin.apply_cli_overrides(cfg, processor_argv + [str(work)])
    cfg["progress_enabled"] = False
    args.include_overhang_wall = plugin.fnum(cfg, "max_overhang_deg") <= -89.9
    return cfg


def audit_pair(
    original: Path,
    processed: Path,
    args: argparse.Namespace,
) -> tuple[int, int, int, audit_coverage.GeneratedSafety, audit_coverage.Continuity, audit_coverage.PassStats]:
    include_overhang = bool(getattr(args, "include_overhang_wall", False))
    orig = audit_coverage.parse_file(str(original), include_overhang)
    proc = audit_coverage.parse_file(str(processed), include_overhang)
    wall_bad, _ = audit_coverage.audit_wall(orig, proc, args.wall_thr, args.per_layer)
    gen_safety = audit_coverage.audit_generated_safety(
        orig,
        proc,
        args.generated_thr,
        max(0.1, args.generated_step),
    )
    surface_bad, top_missing, _, pass_stats = audit_coverage.audit_surface(
        orig,
        proc,
        args.surface_thr,
        args.surface_step,
        args.per_layer_surface,
        args.require_top_finish,
        args.pass_radius,
    )
    return (
        wall_bad,
        surface_bad,
        top_missing,
        gen_safety,
        proc.continuity,
        pass_stats,
    )


def validate_one(path: Path, args: argparse.Namespace) -> ValidationResult:
    marker_counts = bowp_marker_counts(path, None if args.audit_processed_self else 20000)
    input_processed = any(marker_counts.values())
    marker_scan_full = args.audit_processed_self
    if not input_processed and is_bowp_processed(path):
        marker_counts = bowp_marker_counts(path, None)
        input_processed = True
        marker_scan_full = True
    if input_processed and not args.audit_processed_self:
        legacy_step = marker_counts.get("BOWP step ironing", 0) > 0
        current_script = marker_counts.get("BOWP script ironing", 0) > 0
        spiral_markers = marker_counts.get("BOWP spiral", 0)
        print(f"file={path}", flush=True)
        print("status=skipped_already_processed", flush=True)
        print("input_already_processed=true", flush=True)
        print(f"source_size_mb={path.stat().st_size / 1024 / 1024:.2f}", flush=True)
        print(f"marker_scan_full={str(marker_scan_full).lower()}", flush=True)
        print(f"legacy_step_ironing={str(legacy_step).lower()}", flush=True)
        print(f"current_script_ironing={str(current_script).lower()}", flush=True)
        print(f"spiral_markers={spiral_markers}", flush=True)
        print("", flush=True)
        return ValidationResult(
            status="skipped_already_processed",
            path=path,
            input_processed=True,
            legacy_step_ironing=legacy_step,
            current_script_ironing=current_script,
            spiral_markers=spiral_markers,
        )

    tmpdir = Path(tempfile.mkdtemp(prefix="bowp-validate-"))
    work = tmpdir / path.name
    print(f"file={path}", flush=True)
    print("status=running", flush=True)
    print(f"source_size_mb={path.stat().st_size / 1024 / 1024:.2f}", flush=True)
    shutil.copyfile(path, work)
    try:
        cfg = build_processor_config(args, work)
        start = time.perf_counter()
        if input_processed:
            seams, stats = 0, plugin.empty_stats()
        else:
            seams, stats = plugin.run_processor(str(work), cfg)
        process_s = time.perf_counter() - start
        start = time.perf_counter()
        wall_bad, surface_bad, top_missing, gen_safety, continuity, pass_stats = audit_pair(path, work, args)
        audit_s = time.perf_counter() - start
        pass_cv_bad = args.max_pass_cv > 0 and pass_stats.cv > args.max_pass_cv
        gen_external_bad = gen_safety.external_bad > args.max_generated_external
        gen_internal_bad = (
            args.max_generated_internal >= 0
            and gen_safety.internal_bad > args.max_generated_internal
        )
        gen_unneeded_ironing_bad = (
            gen_safety.unneeded_ironing_bad > args.max_generated_unneeded_ironing
        )
        fail = (
            wall_bad
            or surface_bad
            or (args.require_top_finish and top_missing)
            or gen_external_bad
            or gen_internal_bad
            or gen_unneeded_ironing_bad
            or continuity.max_feed_ratio > args.max_feed_ratio
            or continuity.max_flow_ratio > args.max_flow_ratio
            or (
                continuity.transition_pairs
                and continuity.max_transition_feed_ratio > args.max_transition_feed_ratio
            )
            or (
                continuity.transition_pairs
                and continuity.max_transition_flow_ratio > args.max_transition_flow_ratio
            )
            or pass_cv_bad
        )
        print(f"status={'fail' if fail else 'ok'}", flush=True)
        print(f"input_already_processed={str(input_processed).lower()}", flush=True)
        print(f"marker_scan_full={str(marker_scan_full).lower()}", flush=True)
        print(f"processor_args_count={len(getattr(args, 'processor_argv', []))}", flush=True)
        print(f"audit_include_overhang_wall={str(getattr(args, 'include_overhang_wall', False)).lower()}", flush=True)
        print(f"legacy_step_ironing={str(marker_counts.get('BOWP step ironing', 0) > 0).lower()}", flush=True)
        print(f"current_script_ironing={str(marker_counts.get('BOWP script ironing', 0) > 0).lower()}", flush=True)
        print(f"spiral_markers={marker_counts.get('BOWP spiral', 0)}", flush=True)
        if input_processed:
            if fail:
                processed_self_status = "processed_output_fails_current_audit"
            else:
                processed_self_status = "processed_output_passes_current_audit"
            print(f"processed_self_status={processed_self_status}", flush=True)
        print(f"seams={seams}", flush=True)
        print(f"process_s={process_s:.3f}", flush=True)
        print(f"audit_s={audit_s:.3f}", flush=True)
        print(f"script_ironing_lines={stats.script_ironing_lines}", flush=True)
        print(f"wall_bad={wall_bad}", flush=True)
        print(f"surface_bad={surface_bad}", flush=True)
        print(f"top_finish_missing={top_missing}", flush=True)
        print(f"generated_external_bad={gen_safety.external_bad}", flush=True)
        print(f"generated_internal_bad={gen_safety.internal_bad}", flush=True)
        print(f"generated_unneeded_ironing_bad={gen_safety.unneeded_ironing_bad}", flush=True)
        print(f"generated_safety_samples={gen_safety.samples}", flush=True)
        print(f"generated_safety_segments={gen_safety.generated_segments}", flush=True)
        print(f"max_generated_external={args.max_generated_external}", flush=True)
        if args.max_generated_internal >= 0:
            print(f"max_generated_internal={args.max_generated_internal}", flush=True)
        print(f"max_generated_unneeded_ironing={args.max_generated_unneeded_ironing}", flush=True)
        print(f"pass_count_min={pass_stats.min_count}", flush=True)
        print(f"pass_count_max={pass_stats.max_count}", flush=True)
        print(f"pass_count_avg={pass_stats.avg:.2f}", flush=True)
        print(f"pass_count_stdev={pass_stats.stdev:.2f}", flush=True)
        print(f"pass_count_cv={pass_stats.cv:.2f}", flush=True)
        print(f"pass_count_samples={pass_stats.samples}", flush=True)
        print(f"pass_count_radius={args.pass_radius:.2f}", flush=True)
        if args.max_pass_cv > 0:
            print(f"max_pass_cv={args.max_pass_cv:.2f}", flush=True)
        print(f"feed_ratio={continuity.max_feed_ratio:.2f}", flush=True)
        print(f"flow_ratio={continuity.max_flow_ratio:.2f}", flush=True)
        print(f"transition_pairs={continuity.transition_pairs}", flush=True)
        print(f"transition_feed_ratio={continuity.max_transition_feed_ratio:.2f}", flush=True)
        print(f"transition_flow_ratio={continuity.max_transition_flow_ratio:.2f}", flush=True)
        print("", flush=True)
        return ValidationResult(
            status="fail" if fail else "ok",
            path=path,
            process_s=process_s,
            audit_s=audit_s,
            wall_bad=wall_bad,
            surface_bad=surface_bad,
            top_missing=top_missing,
            generated_external_bad=gen_safety.external_bad,
            generated_internal_bad=gen_safety.internal_bad,
            generated_unneeded_ironing_bad=gen_safety.unneeded_ironing_bad,
            generated_safety_samples=gen_safety.samples,
            feed_ratio=continuity.max_feed_ratio,
            flow_ratio=continuity.max_flow_ratio,
            transition_feed_ratio=continuity.max_transition_feed_ratio,
            transition_flow_ratio=continuity.max_transition_flow_ratio,
            pass_cv=pass_stats.cv,
            script_ironing_lines=stats.script_ironing_lines,
            input_processed=input_processed,
            legacy_step_ironing=marker_counts.get("BOWP step ironing", 0) > 0,
            current_script_ironing=marker_counts.get("BOWP script ironing", 0) > 0,
            spiral_markers=marker_counts.get("BOWP spiral", 0),
        )
    finally:
        if args.keep_temp:
            print(f"temp_dir={tmpdir}", flush=True)
        else:
            if not cleanup_temp_dir(tmpdir):
                print(f"temp_cleanup_failed={tmpdir}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Batch validate BOWP processing using temporary G-code copies.")
    parser.add_argument("paths", nargs="*", help="Explicit G-code files to validate")
    parser.add_argument("--root", default=None, help="Directory to discover G-code files from")
    parser.add_argument("--recursive", action="store_true", help="Discover G-code files recursively under --root")
    parser.add_argument("--limit", type=int, default=None, help="Limit discovered files, sorted by size")
    parser.add_argument("--min-size-mb", type=float, default=None, help="Only discover files at or above this size")
    parser.add_argument("--max-size-mb", type=float, default=None, help="Only discover files at or below this size")
    parser.add_argument("--wall-thr", type=float, default=0.9)
    parser.add_argument("--surface-thr", type=float, default=0.9)
    parser.add_argument("--generated-thr", type=float, default=0.9)
    parser.add_argument("--generated-step", type=float, default=0.8)
    parser.add_argument("--max-generated-external", type=int, default=0)
    parser.add_argument("--max-generated-internal", type=int, default=-1)
    parser.add_argument("--max-generated-unneeded-ironing", type=int, default=0)
    parser.add_argument(
        "--processor-command-file",
        default=None,
        help="Read processor CLI options from a saved command file such as orca_postprocess_command.txt",
    )
    parser.add_argument(
        "--processor-args",
        default="",
        help="Processor CLI options to apply before each temporary G-code path",
    )
    parser.add_argument("--per-layer", type=int, default=300)
    parser.add_argument("--per-layer-surface", type=int, default=800)
    parser.add_argument("--surface-step", type=float, default=1.0)
    parser.add_argument("--pass-radius", type=float, default=0.25)
    parser.add_argument("--max-feed-ratio", type=float, default=4.0)
    parser.add_argument("--max-flow-ratio", type=float, default=8.0)
    parser.add_argument("--max-transition-feed-ratio", type=float, default=4.0)
    parser.add_argument("--max-transition-flow-ratio", type=float, default=8.0)
    parser.add_argument(
        "--max-pass-cv",
        type=float,
        default=0.0,
        help="Optional failure threshold for pass-count coefficient of variation; 0 disables",
    )
    parser.add_argument("--require-top-finish", action="store_true")
    parser.add_argument("--audit-processed-self", action="store_true", help="Audit already processed files against themselves")
    parser.add_argument("--keep-temp", action="store_true")
    parser.add_argument("--no-summary", action="store_true", help="Do not print final batch summary")
    args = parser.parse_args()
    processor_argv: list[str] = []
    if args.processor_command_file:
        processor_argv.extend(processor_args_from_command_text(Path(args.processor_command_file).read_text(encoding="utf-8")))
    if args.processor_args:
        processor_argv.extend(processor_args_from_command_text(args.processor_args))
    args.processor_argv = processor_argv

    files = [Path(p).resolve() for p in args.paths]
    if args.root:
        files.extend(discover_files(
            Path(args.root).resolve(),
            args.limit,
            args.min_size_mb,
            args.max_size_mb,
            args.recursive,
        ))
    if not files:
        raise SystemExit("no input files; pass paths or --root")

    results: list[ValidationResult] = []
    for path in files:
        if not path.exists():
            print(f"file={path}", flush=True)
            print("status=missing", flush=True)
            print("", flush=True)
            results.append(ValidationResult(status="missing", path=path))
            continue
        results.append(validate_one(path, args))
    failed = sum(1 for result in results if result.status in {"fail", "missing"})
    if not args.no_summary:
        total_process_s = sum(result.process_s for result in results)
        total_audit_s = sum(result.audit_s for result in results)
        total_wall_bad = sum(result.wall_bad for result in results)
        total_surface_bad = sum(result.surface_bad for result in results)
        total_top_missing = sum(result.top_missing for result in results)
        total_generated_external = sum(result.generated_external_bad for result in results)
        total_generated_internal = sum(result.generated_internal_bad for result in results)
        total_generated_unneeded_ironing = sum(result.generated_unneeded_ironing_bad for result in results)
        total_generated_safety_samples = sum(result.generated_safety_samples for result in results)
        total_script_ironing = sum(result.script_ironing_lines for result in results)
        ok_count = sum(1 for result in results if result.status == "ok")
        skipped_count = sum(1 for result in results if result.status == "skipped_already_processed")
        missing_count = sum(1 for result in results if result.status == "missing")
        fail_count = sum(1 for result in results if result.status == "fail")
        print("batch_summary=begin", flush=True)
        print(f"batch_total={len(results)}", flush=True)
        print(f"batch_ok={ok_count}", flush=True)
        print(f"batch_fail={fail_count}", flush=True)
        print(f"batch_skipped_already_processed={skipped_count}", flush=True)
        print(f"batch_missing={missing_count}", flush=True)
        print(f"batch_recursive={str(args.recursive).lower()}", flush=True)
        print(f"batch_process_s={total_process_s:.3f}", flush=True)
        print(f"batch_audit_s={total_audit_s:.3f}", flush=True)
        print(f"batch_wall_bad={total_wall_bad}", flush=True)
        print(f"batch_surface_bad={total_surface_bad}", flush=True)
        print(f"batch_top_finish_missing={total_top_missing}", flush=True)
        print(f"batch_generated_external_bad={total_generated_external}", flush=True)
        print(f"batch_generated_internal_bad={total_generated_internal}", flush=True)
        print(f"batch_generated_unneeded_ironing_bad={total_generated_unneeded_ironing}", flush=True)
        print(f"batch_generated_safety_samples={total_generated_safety_samples}", flush=True)
        print(f"batch_script_ironing_lines={total_script_ironing}", flush=True)
        if results:
            worst_feed = max(results, key=lambda row: row.feed_ratio)
            worst_flow = max(results, key=lambda row: row.flow_ratio)
            worst_transition_feed = max(results, key=lambda row: row.transition_feed_ratio)
            worst_transition_flow = max(results, key=lambda row: row.transition_flow_ratio)
            worst_pass_cv = max(results, key=lambda row: row.pass_cv)
            print(f"batch_max_feed_ratio={worst_feed.feed_ratio:.2f}", flush=True)
            print(f"batch_max_feed_file={worst_feed.path}", flush=True)
            print(f"batch_max_flow_ratio={worst_flow.flow_ratio:.2f}", flush=True)
            print(f"batch_max_flow_file={worst_flow.path}", flush=True)
            print(f"batch_max_transition_feed_ratio={worst_transition_feed.transition_feed_ratio:.2f}", flush=True)
            print(f"batch_max_transition_feed_file={worst_transition_feed.path}", flush=True)
            print(f"batch_max_transition_flow_ratio={worst_transition_flow.transition_flow_ratio:.2f}", flush=True)
            print(f"batch_max_transition_flow_file={worst_transition_flow.path}", flush=True)
            print(f"batch_max_pass_cv={worst_pass_cv.pass_cv:.2f}", flush=True)
            print(f"batch_max_pass_cv_file={worst_pass_cv.path}", flush=True)
        print("batch_summary=end", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
