#!/usr/bin/env python3
"""Plot recorded CARLA trials without simulator access or modifying raw JSON.

Usage: python3 tools/plot_carla_closed_loop_validation.py
The default inputs are the four byte-preserved reports in docs/validation/
carla-closed-loop-20260922. Derived measurements are written to analysis.json.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


FRAME_SECONDS = 0.05
STOP_THRESHOLD_MPS = 0.15
CASES = ("vehicle_stop", "vehicle_avoidance", "pedestrian_stop")
TITLES = ("Roadblock: stop passed", "Single car: no avoidance", "Pedestrian: collision")
COLORS = ("#137957", "#a86608", "#bb3545")


def stop_intervals(samples):
    """Observed sub-threshold runs after meaningful forward motion, excluding launch."""
    runs, current, peak = [], [], 0.0
    for sample in samples:
        peak = max(peak, sample["speed_mps"])
        if sample["speed_mps"] < STOP_THRESHOLD_MPS and sample["progress_m"] > 5 and peak >= 2:
            current.append(sample)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return [{"start_s": run[0]["sim_s"], "end_s": run[-1]["sim_s"],
             "duration_s": run[-1]["sim_s"] - run[0]["sim_s"],
             "start_frame": run[0]["frame"], "end_frame": run[-1]["frame"],
             "start_progress_m": run[0]["progress_m"], "end_progress_m": run[-1]["progress_m"],
             "start_clearance_m": run[0]["clearance_m"], "end_clearance_m": run[-1]["clearance_m"]}
            for run in runs]


def frame_time(report, frame):
    first = report["samples"][0]
    return first["sim_s"] + (frame - first["frame"]) * FRAME_SECONDS


def analyse(reports):
    analysis = {"frame_seconds": FRAME_SECONDS, "stop_threshold_mps": STOP_THRESHOLD_MPS,
                "timing_note": "Frame differences use the recorded 0.05 s step. Do not subtract ROS event times from CARLA sensor timestamps.",
                "raw_time_domains": {
                    "samples[].sim_s": "Elapsed CARLA world snapshot time from trial loop start",
                    "events[].sim_time": "ROS node clock (absolute)",
                    "collisions[].timestamp": "CARLA collision event timestamp (absolute)",
                    "pedestrian_states.*.trigger_time/stop_time": "CARLA snapshot timestamp (absolute)",
                    "ROS message header/stamp fields": "ROS clock"}, "cases": {}}
    for case, report in reports.items():
        samples = report["samples"]
        for before, after in zip(samples, samples[1:]):
            observed = after["sim_s"] - before["sim_s"]
            expected = (after["frame"] - before["frame"]) * FRAME_SECONDS
            if not math.isclose(observed, expected, abs_tol=1e-6):
                raise ValueError(f"{case}: sample timestamps do not match the expected frame step")
        analysis["cases"][case] = {
            "result": report["result"], "speed_cap_kmh": report["speed_cap_kmh"],
            "observed_peak_kmh": max(s["speed_mps"] for s in samples) * 3.6,
            "sample_end_s": samples[-1]["sim_s"], "max_progress_m": report["max_progress_m"],
            "min_clearance_m": report["min_clearance_m"], "final_clearance_m": samples[-1]["clearance_m"],
            "max_lateral_m": report["max_lateral_m"], "collision_records": len(report["collisions"]),
            "stop_intervals": stop_intervals(samples),
        }
    roadblock = analysis["cases"]["vehicle_stop"]
    first_pause, final_stop = roadblock["stop_intervals"][0], roadblock["stop_intervals"][-1]
    creep = [s for s in reports["vehicle_stop"]["samples"]
             if first_pause["end_frame"] <= s["frame"] <= final_stop["start_frame"]]
    roadblock["creep_after_first_pause"] = {
        "duration_s": final_stop["start_s"] - first_pause["end_s"],
        "progress_m": final_stop["start_progress_m"] - first_pause["end_progress_m"],
        "peak_kmh": max(s["speed_mps"] for s in creep) * 3.6}

    report = reports["pedestrian_stop"]
    state = next(iter(report["pedestrian_states"].values()))
    collision = min(report["collisions"], key=lambda record: record["frame"])
    first_collision_frame = collision["frame"]
    yaw = math.radians(report["test_start"]["rpy"][2])
    start = report["test_start"]["xyz"]
    right = (-math.sin(yaw), math.cos(yaw))

    def approach_lateral(sample):
        position = sample["hazard_xyz"][0]
        return (position[0] - start[0]) * right[0] + (position[1] - start[1]) * right[1]

    # This is a straight approach-line calculation, not a new map/lane inference.
    before_impact = [s for s in report["samples"] if s["frame"] < first_collision_frame]
    near_path = next(s for s in before_impact if abs(approach_lateral(s)) < 0.5)
    stopped = min(before_impact, key=lambda s: abs(s["frame"] - state["stop_frame"]))
    last_before = before_impact[-1]
    analysis["cases"]["pedestrian_stop"]["pedestrian_evidence"] = {
        "trigger_frame": state["trigger_frame"], "stop_frame": state["stop_frame"],
        "first_collision_frame": first_collision_frame,
        "trigger_to_collision_s": (first_collision_frame - state["trigger_frame"]) * FRAME_SECONDS,
        "pedestrian_stop_to_collision_s": (first_collision_frame - state["stop_frame"]) * FRAME_SECONDS,
        "trigger_elapsed_s": frame_time(report, state["trigger_frame"]),
        "stop_elapsed_s": frame_time(report, state["stop_frame"]),
        "collision_elapsed_s": frame_time(report, first_collision_frame),
        "first_sample_within_0_5m_of_approach_line_frame": near_path["frame"],
        "stopped_sample_frame": stopped["frame"],
        "stopped_sample_signed_approach_lateral_m": approach_lateral(stopped),
        "stopped_sample_carla_xyz": stopped["hazard_xyz"][0],
        "last_pre_collision_sample_frame": last_before["frame"],
        "last_pre_collision_speed_kmh": last_before["speed_mps"] * 3.6,
        "last_pre_collision_brake": last_before["brake"],
        "nonempty_predicted_object_samples": sum(bool(s["objects_by_class"]) for s in report["samples"]),
        "total_samples": len(report["samples"]),
    }
    return analysis


def plot(reports, analysis, output):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.titleweight": "bold", "axes.labelcolor": "#334155"})
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5), sharey="row")
    fig.subplots_adjust(left=.065, right=.985, bottom=.14, top=.83, hspace=.42, wspace=.18)
    fig.suptitle("CARLA Town05 | Closed-loop validation", x=.065, y=.965, ha="left", fontsize=21, fontweight="bold")
    fig.text(.065, .919, "22 September 2026   |   Configured cap: 20 km/h   |   Observed peaks remained below 20 km/h", fontsize=12, color="#475569")
    for column, (case, title, color) in enumerate(zip(CASES, TITLES, COLORS)):
        report, metrics = reports[case], analysis["cases"][case]
        samples = report["samples"]
        top, bottom = axes[:, column]
        top.set_title(title, loc="left", color=color, pad=13, fontsize=13)
        top.plot([s["progress_m"] for s in samples], [s["speed_mps"] * 3.6 for s in samples], color=color, lw=2.2)
        top.axhline(report["speed_cap_kmh"], color="#64748b", lw=1, ls="--")
        top.text(1.2, 20.6, "20 km/h cap", color="#64748b", fontsize=9)
        top.text(.96, .75, f"Peak {metrics['observed_peak_kmh']:.2f} km/h", transform=top.transAxes,
                 ha="right", color=color, fontweight="bold")
        top.set(xlim=(-1, 46), ylim=(-.7, 23), xlabel="Forward progress (m)")
        bottom.plot([s["sim_s"] for s in samples], [s["clearance_m"] for s in samples], color=color, lw=2.2)
        bottom.set(xlim=(0, 46), ylim=(-1.3, 46), xlabel="Elapsed simulation time (s)")
        for ax in (top, bottom):
            ax.grid(alpha=.16)
            ax.set_axisbelow(True)
        if column == 0:
            top.set_ylabel("Ego speed (km/h)")
            bottom.set_ylabel("Nearest hazard outline clearance (m)")
        if case == "vehicle_stop":
            stop = metrics["stop_intervals"][-1]
            bottom.axvspan(stop["start_s"], stop["end_s"], color=color, alpha=.14)
            bottom.annotate("Final 2.0 s stop\n4.744 m clearance", xy=(stop["end_s"], stop["end_clearance_m"]),
                            xytext=(24, 17), color=color, arrowprops={"arrowstyle": "->", "color": color})
            top.scatter(samples[-1]["progress_m"], samples[-1]["speed_mps"] * 3.6, color=color, s=35, zorder=4)
            top.text(.04, .08, "Brief pauses, then creep\n+1.59 m before final stop", transform=top.transAxes, fontsize=9,
                     bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
        elif case == "vehicle_avoidance":
            bottom.annotate("45 s timeout\n4.952 m final clearance", xy=(samples[-1]["sim_s"], samples[-1]["clearance_m"]),
                            xytext=(22, 18), color=color, arrowprops={"arrowstyle": "->", "color": color})
            top.text(.04, .08, "No completed pass\nMax lateral offset: 0.052 m", transform=top.transAxes, fontsize=9,
                     bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
        else:
            evidence = metrics["pedestrian_evidence"]
            collision_time = evidence["collision_elapsed_s"]
            bottom.axvline(collision_time, color=color, lw=1.2, ls="--", ymax=.75)
            bottom.annotate(f"First collision: {collision_time:.2f} s\n4.35 s after trigger", xy=(collision_time, 0),
                            xytext=(20, 20), color=color, arrowprops={"arrowstyle": "->", "color": color})
            bottom.text(.04, .91, "Pedestrian on approach line\n2.50 s before impact", transform=bottom.transAxes,
                        va="top", fontsize=9, bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
            # Mark the actual recorded sample where the collision was detected,
            # without inventing an unrecorded speed at the collision event frame.
            top.scatter(samples[-1]["progress_m"], samples[-1]["speed_mps"] * 3.6,
                        marker="X", color=color, edgecolors="white", s=85, zorder=4)
            top.text(.04, .08, "Collision detected (X)\nLast pre-impact: 19.11 km/h", transform=top.transAxes, fontsize=9,
                     bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
    baseline = analysis["cases"]["baseline"]
    fig.text(.065, .070, f"Baseline (not plotted): passed, {baseline['max_progress_m']:.2f} m travel; peak {baseline['observed_peak_kmh']:.2f} km/h.  Stop criterion: speed < 0.15 m/s for 2 s.", fontsize=10, color="#475569")
    fig.text(.065, .040, "Raw samples are unchanged. Collision timing uses frame differences at 0.05 s/frame; absolute CARLA and ROS timestamps are not mixed.", fontsize=9, color="#64748b")
    fig.savefig(output, dpi=160, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = Path(__file__).resolve().parents[1] / "docs/validation/carla-closed-loop-20260922"
    parser.add_argument("--input-dir", type=Path, default=default)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    reports = {case: json.loads((args.input_dir / (case + ".json")).read_text())
               for case in ("baseline",) + CASES}
    analysis = analyse(reports)
    analysis["raw_sha256"] = {
        case + ".json": hashlib.sha256((args.input_dir / (case + ".json")).read_bytes()).hexdigest()
        for case in reports}
    output = args.output or args.input_dir / "summary.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    plot(reports, analysis, output)
    output.with_name("analysis.json").write_text(json.dumps(analysis, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
