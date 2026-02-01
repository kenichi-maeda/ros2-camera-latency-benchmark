#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import statistics


def read_latencies(csv_path):
    latencies = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                latencies.append(float(row["latency_ms"]))
            except (KeyError, ValueError):
                continue
    return latencies


def percentile(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    if p <= 0:
        return sorted_vals[0]
    if p >= 100:
        return sorted_vals[-1]
    idx = int(round((p / 100.0) * (len(sorted_vals) - 1)))
    return sorted_vals[idx]


def summarize(latencies):
    if not latencies:
        return {
            "count": 0,
            "mean": 0.0,
            "p50": 0.0,
            "p95": 0.0,
            "p99": 0.0,
            "min": 0.0,
            "max": 0.0,
        }
    sorted_vals = sorted(latencies)
    return {
        "count": len(sorted_vals),
        "mean": statistics.mean(sorted_vals),
        "p50": percentile(sorted_vals, 50),
        "p95": percentile(sorted_vals, 95),
        "p99": percentile(sorted_vals, 99),
        "min": sorted_vals[0],
        "max": sorted_vals[-1],
    }


def format_row(name, stats):
    return (
        f"{name:<16}  "
        f"{stats['count']:>7}  "
        f"{stats['mean']:>8.3f}  "
        f"{stats['p50']:>8.3f}  "
        f"{stats['p95']:>8.3f}  "
        f"{stats['p99']:>8.3f}  "
        f"{stats['min']:>8.3f}  "
        f"{stats['max']:>8.3f}"
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("dir", nargs="?", default=".", help="Results directory (default: .)")
    return parser.parse_args()


def get_csv_files(directory):
    pattern = os.path.join(directory, "*.csv")
    return sorted(glob.glob(pattern))


def print_table(files):
    header = (
        f"{'case':<16}  {'count':>7}  {'mean':>8}  {'p50':>8}  "
        f"{'p95':>8}  {'p99':>8}  {'min':>8}  {'max':>8}"
    )
    print(header)
    print("-" * len(header))

    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        stats = summarize(read_latencies(path))
        print(format_row(name, stats))


def run(directory):
    files = get_csv_files(directory)
    if not files:
        print(f"No CSV files found in {directory}")
        return
    print_table(files)


def main():
    args = parse_args()
    run(args.dir)


if __name__ == "__main__":
    main()
