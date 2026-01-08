#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import os
import re
from collections import defaultdict
from pathlib import Path

SHOW_PARSER = argparse.ArgumentParser(add_help=False)
SHOW_PARSER.add_argument("--show", action="store_true")
SHOW_ARGS, _ = SHOW_PARSER.parse_known_args()
SHOW_ENABLED = SHOW_ARGS.show and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

import matplotlib

if not SHOW_ENABLED:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_RESIDUAL_THRESHOLD = 0.012
DEFAULT_PLOT_RESIDUAL_THRESHOLD = 0.013


def find_latest_log(log_dir: Path) -> Path:
    candidates = sorted(log_dir.glob("runtime_log_*.txt"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No runtime_log_*.txt found in {log_dir}")
    return candidates[-1]


def extract_log_timestamp(log_path: Path) -> str:
    match = re.search(r"runtime_log_(\d{8}_\d{6})", log_path.stem)
    if match:
        return match.group(1)
    return log_path.stem


def find_residual_segments(times_sec, avg_residuals, threshold):
    if not times_sec:
        return []
    segments = []
    above = avg_residuals[0] > threshold
    start = times_sec[0] if above else None
    for idx in range(1, len(times_sec)):
        v0 = avg_residuals[idx - 1]
        v1 = avg_residuals[idx]
        t0 = times_sec[idx - 1]
        t1 = times_sec[idx]
        if not above and v1 > threshold:
            t_cross = t0 if v1 == v0 else t0 + (threshold - v0) * (t1 - t0) / (v1 - v0)
            start = t_cross
            above = True
        elif above and v1 <= threshold:
            t_cross = t0 if v1 == v0 else t0 + (threshold - v0) * (t1 - t0) / (v1 - v0)
            segments.append((start, t_cross))
            above = False
    if above and start is not None:
        segments.append((start, times_sec[-1]))
    return segments


def parse_log(log_path: Path):
    image_re = re.compile(r"\[(Image Use|Image Drop)\].*utc=(\d{8}_\d{6})")
    lio_re = re.compile(
        r"\[LIO Degenerate\].*utc=(\d{8}_\d{6}).*avg_residual=([0-9.eE+-]+).*degenerate=(\d)"
    )
    thresh_re = re.compile(r"thresh_residual=([0-9.eE+-]+)")
    vio_re = re.compile(r"\[VIO Prune\].*utc=(\d{8}_\d{6}).*reason=([a-z_]+)")

    stats = defaultdict(
        lambda: {
            "image_total": 0,
            "image_drop": 0,
            "lio_total": 0,
            "lio_degenerate": 0,
            "lio_residual_sum": 0.0,
            "lio_residual_count": 0,
            "lio_residual_max": 0.0,
            "vio_total": 0,
            "vio_max_points": 0,
            "vio_age": 0,
        }
    )

    residual_threshold = None

    with log_path.open("r") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            image_match = image_re.search(line)
            if image_match:
                label, utc = image_match.groups()
                stats[utc]["image_total"] += 1
                if label == "Image Drop":
                    stats[utc]["image_drop"] += 1
                continue
            lio_match = lio_re.search(line)
            if lio_match:
                utc, avg_residual, degenerate = lio_match.groups()
                avg_residual = float(avg_residual)
                stats[utc]["lio_total"] += 1
                stats[utc]["lio_residual_sum"] += avg_residual
                stats[utc]["lio_residual_count"] += 1
                stats[utc]["lio_residual_max"] = max(stats[utc]["lio_residual_max"], avg_residual)
                if degenerate == "1":
                    stats[utc]["lio_degenerate"] += 1
                if residual_threshold is None:
                    thresh_match = thresh_re.search(line)
                    if thresh_match:
                        residual_threshold = float(thresh_match.group(1))
                continue
            vio_match = vio_re.search(line)
            if vio_match:
                utc, reason = vio_match.groups()
                stats[utc]["vio_total"] += 1
                if reason == "max_points":
                    stats[utc]["vio_max_points"] += 1
                elif reason == "age":
                    stats[utc]["vio_age"] += 1
                continue

    if not stats:
        raise ValueError(f"No runtime log entries parsed in {log_path}")

    ordered_keys = sorted(stats.keys())
    start_time = dt.datetime.strptime(ordered_keys[0], "%Y%m%d_%H%M%S")
    end_time = dt.datetime.strptime(ordered_keys[-1], "%Y%m%d_%H%M%S")
    ordered_times = []
    current_time = start_time
    while current_time <= end_time:
        ordered_times.append(current_time)
        current_time += dt.timedelta(seconds=1)

    return stats, ordered_times, start_time, residual_threshold


def write_csv(log_path: Path, stats, ordered_times, start_time):
    csv_path = log_path.with_name(log_path.stem + "_stats.csv")
    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "utc",
                "seconds_from_start",
                "image_total",
                "image_drop",
                "image_drop_rate",
                "lio_total",
                "lio_degenerate",
                "lio_degenerate_ratio",
                "lio_avg_residual",
                "lio_max_residual",
                "vio_prune_total",
                "vio_prune_max_points",
                "vio_prune_age",
            ]
        )
        for current_time in ordered_times:
            utc = current_time.strftime("%Y%m%d_%H%M%S")
            row = stats.get(
                utc,
                {
                    "image_total": 0,
                    "image_drop": 0,
                    "lio_total": 0,
                    "lio_degenerate": 0,
                    "lio_residual_sum": 0.0,
                    "lio_residual_count": 0,
                    "lio_residual_max": 0.0,
                    "vio_total": 0,
                    "vio_max_points": 0,
                    "vio_age": 0,
                },
            )
            image_total = row["image_total"]
            image_drop = row["image_drop"]
            lio_total = row["lio_total"]
            lio_degenerate = row["lio_degenerate"]
            vio_total = row["vio_total"]
            drop_rate = image_drop / image_total if image_total else 0.0
            lio_ratio = lio_degenerate / lio_total if lio_total else 0.0
            lio_residual_count = row["lio_residual_count"]
            lio_avg_residual = (
                row["lio_residual_sum"] / lio_residual_count if lio_residual_count else 0.0
            )
            seconds_from_start = int((current_time - start_time).total_seconds())
            writer.writerow(
                [
                    utc,
                    seconds_from_start,
                    image_total,
                    image_drop,
                    f"{drop_rate:.6f}",
                    lio_total,
                    lio_degenerate,
                    f"{lio_ratio:.6f}",
                    f"{lio_avg_residual:.6f}",
                    f"{row['lio_residual_max']:.6f}",
                    vio_total,
                    row["vio_max_points"],
                    row["vio_age"],
                ]
            )
    return csv_path


def write_plot(
    log_path: Path,
    stats,
    ordered_times,
    start_time,
    summary,
    show,
    residual_threshold,
    plot_threshold,
):
    times_sec = []
    drop_rates = []
    vio_counts = []
    avg_residuals = []
    for current_time in ordered_times:
        utc = current_time.strftime("%Y%m%d_%H%M%S")
        row = stats.get(
            utc,
            {
                "image_total": 0,
                "image_drop": 0,
                "lio_total": 0,
                "lio_degenerate": 0,
                "lio_residual_sum": 0.0,
                "lio_residual_count": 0,
                "vio_total": 0,
            },
        )
        image_total = row["image_total"]
        image_drop = row["image_drop"]
        drop_rates.append(image_drop / image_total if image_total else 0.0)
        vio_counts.append(row["vio_total"])
        lio_residual_count = row["lio_residual_count"]
        avg_residuals.append(
            row["lio_residual_sum"] / lio_residual_count if lio_residual_count else 0.0
        )
        times_sec.append((current_time - start_time).total_seconds())

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(times_sec, drop_rates, color="#d95f02")
    axes[0].set_ylabel("Drop rate")
    axes[0].set_ylim(0, 1)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times_sec, avg_residuals, color="#66a61e")
    axes[1].axhline(plot_threshold, color="#000000", linestyle="--", linewidth=1.8)
    segments = find_residual_segments(times_sec, avg_residuals, plot_threshold)
    for start, end in segments:
        for ax in axes:
            ax.axvline(start, color="#d7301f", alpha=0.6, linewidth=0.8)
            ax.axvline(end, color="#d7301f", alpha=0.6, linewidth=0.8)
    axes[1].fill_between(
        times_sec,
        plot_threshold,
        avg_residuals,
        where=[value > plot_threshold for value in avg_residuals],
        color="#b2df8a",
        alpha=0.35,
        interpolate=True,
    )
    axes[1].set_ylabel("Avg residual")
    axes[1].grid(True, alpha=0.3)

    axes[2].bar(times_sec, vio_counts, color="#7570b3", width=0.8)
    axes[2].set_ylabel("VIO prune count")
    axes[2].set_xlabel("Seconds from start")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(extract_log_timestamp(log_path))
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    plot_path = log_path.with_name(log_path.stem + "_plot.png")
    fig.savefig(plot_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)
    return plot_path


def summarize_stats(stats, ordered_times, start_time, residual_threshold):
    drop_rates = []
    lio_ratios = []
    avg_residuals = []
    vio_counts = []
    for current_time in ordered_times:
        utc = current_time.strftime("%Y%m%d_%H%M%S")
        row = stats.get(
            utc,
            {
                "image_total": 0,
                "image_drop": 0,
                "lio_total": 0,
                "lio_degenerate": 0,
                "lio_residual_sum": 0.0,
                "lio_residual_count": 0,
                "vio_total": 0,
            },
        )
        image_total = row["image_total"]
        image_drop = row["image_drop"]
        lio_total = row["lio_total"]
        lio_degenerate = row["lio_degenerate"]
        lio_residual_count = row["lio_residual_count"]
        avg_residual = row["lio_residual_sum"] / lio_residual_count if lio_residual_count else 0.0
        drop_rates.append(image_drop / image_total if image_total else 0.0)
        lio_ratios.append(lio_degenerate / lio_total if lio_total else 0.0)
        avg_residuals.append(avg_residual)
        vio_counts.append(row["vio_total"])

    seconds = len(ordered_times)
    drop_rate_avg = sum(drop_rates) / seconds if seconds else 0.0
    drop_rate_max = max(drop_rates) if drop_rates else 0.0
    lio_ratio_avg = sum(lio_ratios) / seconds if seconds else 0.0
    lio_ratio_max = max(lio_ratios) if lio_ratios else 0.0
    vio_total = sum(vio_counts)
    vio_max = max(vio_counts) if vio_counts else 0
    degenerate_seconds = sum(1 for value in avg_residuals if value > residual_threshold)
    degenerate_time_ratio = degenerate_seconds / seconds if seconds else 0.0

    return {
        "seconds": seconds,
        "drop_rate_avg": drop_rate_avg,
        "drop_rate_max": drop_rate_max,
        "lio_ratio_avg": lio_ratio_avg,
        "lio_ratio_max": lio_ratio_max,
        "degenerate_seconds": degenerate_seconds,
        "degenerate_time_ratio": degenerate_time_ratio,
        "vio_total": vio_total,
        "vio_max": vio_max,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument("--log-dir", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--show", action="store_true", help="Display plot window")
    parser.add_argument(
        "--residual-threshold",
        type=float,
        default=None,
        help="Override avg_residual threshold",
    )
    parser.add_argument(
        "--plot-residual-threshold",
        type=float,
        default=None,
        help="Override plot residual threshold",
    )
    args = parser.parse_args()

    log_path = args.log
    if log_path is None:
        log_path = find_latest_log(args.log_dir)
    if not log_path.exists():
        raise FileNotFoundError(f"Missing log file: {log_path}")

    stats, ordered_times, start_time, _ = parse_log(log_path)
    residual_threshold = args.residual_threshold if args.residual_threshold is not None else DEFAULT_RESIDUAL_THRESHOLD
    plot_threshold = (
        args.plot_residual_threshold
        if args.plot_residual_threshold is not None
        else DEFAULT_PLOT_RESIDUAL_THRESHOLD
    )
    summary = summarize_stats(stats, ordered_times, start_time, residual_threshold)
    csv_path = write_csv(log_path, stats, ordered_times, start_time)
    plot_path = write_plot(
        log_path,
        stats,
        ordered_times,
        start_time,
        summary,
        SHOW_ENABLED,
        residual_threshold,
        plot_threshold,
    )
    if args.show and not SHOW_ENABLED:
        print("Display not available; saved plot instead of showing it.")
    print(f"Wrote {csv_path}")
    print(f"Wrote {plot_path}")
    print(f"覆盖秒数: {summary['seconds']}")
    print(f"丢帧率: 均值 {summary['drop_rate_avg']:.3f}, 最大 {summary['drop_rate_max']:.3f}")
    print(f"退化比例: 均值 {summary['lio_ratio_avg']:.3f}, 最大 {summary['lio_ratio_max']:.3f}")
    print(
        f"退化时间占比(残差阈值>{residual_threshold:.3f}):"
        f" {summary['degenerate_time_ratio']:.3f}"
        f" ({summary['degenerate_seconds']}/{summary['seconds']})"
    )
    print(f"VIO 剔除: 总计 {summary['vio_total']}, 单秒最大 {summary['vio_max']}")


if __name__ == "__main__":
    main()
