#!/usr/bin/env python3
"""Low-overhead system/process telemetry for one field experiment.

The monitor intentionally uses only procfs/sysfs and the Python standard
library.  It emits one JSON record per interval so sensor/control timing spikes
can be compared with per-core saturation, clocks, thermals, memory pressure,
storage I/O, network loss and individual process scheduling.
"""

import argparse
import datetime
import json
import os
import re
import signal
import time
from pathlib import Path


PROCESS_PATTERNS = {
    "livox": re.compile(r"livox_ros_driver2_node|livox_ros_drive"),
    "fastlio": re.compile(r"fastlio_mapping"),
    "continuous_localizer": re.compile(r"continuous_map_localizer"),
    "cloud_transformer": re.compile(r"calibrated_cloud_transformer"),
    "nav2_controller": re.compile(r"controller_server"),
    "velocity_smoother": re.compile(r"velocity_smoother"),
    "collision_monitor": re.compile(r"collision_monitor"),
    "runtime_manager": re.compile(r"patrol_runtime_manager"),
    "follower": re.compile(r"waypoint_follower"),
    "safe": re.compile(r"unitree_safe_cmd_node"),
    "sender": re.compile(r"cmd_vel_udp_sender"),
    "sdk_receiver": re.compile(r"go2_sdk2_udp_receiver"),
    "rosbag": re.compile(r"ros2 bag record"),
    "video": re.compile(r"z1pro_capture|gst-launch|ffmpeg"),
    "route_recorder": re.compile(r"route_recorder"),
    "experiment_telemetry": re.compile(r"go2_experiment_telemetry"),
    "performance_monitor": re.compile(r"patrol_performance_monitor"),
}


class RotatingTextOutput:
    """Append line-oriented telemetry with bounded local retention."""

    def __init__(self, path, max_bytes, retained_files):
        self.path = Path(path)
        self.max_bytes = int(max_bytes)
        self.retained_files = int(retained_files)
        if not 1048576 <= self.max_bytes <= 1073741824:
            raise ValueError("rotate-bytes must be within [1 MiB, 1 GiB]")
        if not 2 <= self.retained_files <= 128:
            raise ValueError("rotate-files must be within [2, 128]")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = None
        self.size = 0
        self.last_fsync = time.monotonic()
        self._open()

    def _open(self):
        self.handle = self.path.open("a", buffering=1, encoding="utf-8")
        try:
            self.size = self.path.stat().st_size
        except OSError:
            self.size = 0

    def _rotate(self):
        self.close()
        oldest = Path(str(self.path) + ".%d" % self.retained_files)
        oldest.unlink(missing_ok=True)
        for index in range(self.retained_files - 1, 0, -1):
            source = Path(str(self.path) + ".%d" % index)
            target = Path(str(self.path) + ".%d" % (index + 1))
            if source.exists():
                os.replace(source, target)
        if self.path.exists():
            os.replace(self.path, Path(str(self.path) + ".1"))
        self._open()

    def write_line(self, line):
        encoded_size = len((line + "\n").encode("utf-8"))
        if self.size and self.size + encoded_size > self.max_bytes:
            self._rotate()
        self.handle.write(line + "\n")
        self.size += encoded_size
        now = time.monotonic()
        if now - self.last_fsync >= 1.0:
            self.handle.flush()
            os.fsync(self.handle.fileno())
            self.last_fsync = now

    def close(self):
        if self.handle is None or self.handle.closed:
            return
        self.handle.flush()
        os.fsync(self.handle.fileno())
        self.handle.close()


def read_text(path, default=""):
    try:
        return Path(path).read_text().strip()
    except (OSError, ValueError):
        return default


def read_cpu_totals():
    line = read_text("/proc/stat").splitlines()
    if not line:
        return 0, 0, 0, 0, 0
    fields = line[0].split()
    if not fields or fields[0] != "cpu":
        return 0, 0, 0, 0, 0
    values = [int(value) for value in fields[1:]]
    values += [0] * max(0, 10 - len(values))
    idle = values[3] + values[4]
    total = sum(values[:8])
    ctxt = 0
    running = 0
    blocked = 0
    for item in line[1:]:
        if item.startswith("ctxt "):
            ctxt = int(item.split()[1])
        elif item.startswith("procs_running "):
            running = int(item.split()[1])
        elif item.startswith("procs_blocked "):
            blocked = int(item.split()[1])
    return total, idle, ctxt, running, blocked


def read_per_cpu_counters():
    result = {}
    for line in read_text("/proc/stat").splitlines():
        fields = line.split()
        if (
            not fields
            or not re.match(r"^cpu[0-9]+$", fields[0])
        ):
            continue
        try:
            values = [int(value) for value in fields[1:]]
        except ValueError:
            continue
        values += [0] * max(0, 8 - len(values))
        result[fields[0]] = {
            "total": sum(values[:8]),
            "idle": values[3] + values[4],
        }
    return result


def per_cpu_percentages(previous, current):
    result = {}
    for label, counters in current.items():
        before = previous.get(label, counters)
        total_delta = max(0, counters["total"] - before["total"])
        idle_delta = max(0, counters["idle"] - before["idle"])
        usage = (
            100.0 * (total_delta - idle_delta) / total_delta
            if total_delta > 0
            else 0.0
        )
        result[label] = round(usage, 1)
    return result


def read_memory():
    result = {}
    for line in read_text("/proc/meminfo").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key not in ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree"):
            continue
        parts = value.split()
        result[key] = int(parts[0]) if parts else 0
    return {
        "available_mib": round(result.get("MemAvailable", 0) / 1024.0, 1),
        "total_mib": round(result.get("MemTotal", 0) / 1024.0, 1),
        "swap_used_mib": round(
            (
                result.get("SwapTotal", 0)
                - result.get("SwapFree", 0)
            )
            / 1024.0,
            1,
        ),
    }


def read_pressure(resource):
    result = {}
    for line in read_text("/proc/pressure/%s" % resource).splitlines():
        fields = line.split()
        if not fields:
            continue
        category = fields[0]
        for field in fields[1:]:
            if not field.startswith("avg10="):
                continue
            try:
                result[category + "_avg10"] = float(field.split("=", 1)[1])
            except ValueError:
                pass
    return result


def online_cpu_ids():
    result = []
    for cpu_path in sorted(
        Path("/sys/devices/system/cpu").glob("cpu[0-9]*"),
        key=lambda item: int(item.name[3:]),
    ):
        online_path = cpu_path / "online"
        if not online_path.exists() or read_text(online_path, "1") == "1":
            result.append(int(cpu_path.name[3:]))
    return result


def cpu_frequencies(cpu_ids):
    by_cpu = {}
    for cpu_id in cpu_ids:
        value = read_text(
            "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq"
            % cpu_id
        )
        try:
            by_cpu["cpu%d" % cpu_id] = round(float(value) / 1000.0, 1)
        except ValueError:
            pass
    values = list(by_cpu.values())
    if not values:
        return {}
    return {
        "min_mhz": round(min(values), 1),
        "max_mhz": round(max(values), 1),
        "avg_mhz": round(sum(values) / len(values), 1),
        "per_cpu_mhz": by_cpu,
    }


def thermal_zones():
    result = {}
    for path in Path("/sys/class/thermal").glob("thermal_zone*"):
        temperature_path = path / "temp"
        try:
            value = float(temperature_path.read_text().strip())
        except (OSError, ValueError):
            continue
        if value > 1000.0:
            value /= 1000.0
        if -40.0 <= value <= 150.0:
            name = read_text(str(path / "type"), path.name)
            key = re.sub(r"[^A-Za-z0-9_.-]+", "_", name) or path.name
            if key in result:
                key = "%s_%s" % (key, path.name)
            result[key] = round(value, 1)
    return result


def accelerator_snapshot():
    candidates = {
        "gpu_load_raw": [
            "/sys/devices/platform/17000000.gpu/load",
            "/sys/devices/gpu.0/load",
        ],
        "gpu_frequency_hz": [
            "/sys/devices/platform/17000000.gpu/devfreq/"
            "17000000.gpu/cur_freq",
            "/sys/devices/gpu.0/devfreq/17000000.gpu/cur_freq",
        ],
        "emc_frequency_hz": [
            "/sys/kernel/debug/bpmp/debug/clk/emc/rate",
        ],
    }
    result = {}
    for label, paths in candidates.items():
        for path in paths:
            value = read_text(path)
            try:
                result[label] = int(value)
                break
            except ValueError:
                continue
    if "gpu_load_raw" in result:
        result["gpu_load_pct"] = round(
            result["gpu_load_raw"] / 10.0,
            1,
        )
    return result


def hwmon_snapshot():
    """Read Jetson INA3221 rails and other standard hwmon sensors."""
    result = {}
    for directory in sorted(Path("/sys/class/hwmon").glob("hwmon*")):
        device_name = read_text(str(directory / "name"), directory.name)
        sensors = {}
        for prefix, scale, suffix in (
            ("power", 1000000.0, "w"),
            ("curr", 1000.0, "a"),
            ("in", 1000.0, "v"),
            ("fan", 1.0, "rpm"),
        ):
            for input_path in sorted(
                directory.glob("%s*_input" % prefix)
            ):
                match = re.match(
                    r"^%s([0-9]+)_input$" % prefix,
                    input_path.name,
                )
                if not match:
                    continue
                index = match.group(1)
                label = read_text(
                    str(directory / ("%s%s_label" % (prefix, index))),
                    "%s%s" % (prefix, index),
                )
                try:
                    value = float(input_path.read_text().strip()) / scale
                except (OSError, ValueError):
                    continue
                key = re.sub(
                    r"[^A-Za-z0-9_.-]+",
                    "_",
                    label,
                )
                sensors["%s_%s" % (key, suffix)] = round(value, 6)
        if sensors:
            key = "%s:%s" % (directory.name, device_name)
            result[key] = sensors
    return result


def filesystem_snapshot(path):
    try:
        stat = os.statvfs(path)
    except OSError:
        return {}
    total = stat.f_blocks * stat.f_frsize
    available = stat.f_bavail * stat.f_frsize
    return {
        "path": path,
        "total_mib": round(total / 1048576.0, 1),
        "available_mib": round(available / 1048576.0, 1),
        "used_pct": round(
            100.0 * (total - available) / total,
            1,
        ) if total else 0.0,
    }


def disk_counters():
    result = {}
    device_pattern = re.compile(
        r"^(?:nvme[0-9]+n[0-9]+|mmcblk[0-9]+|sd[a-z]+)$"
    )
    for line in read_text("/proc/diskstats").splitlines():
        fields = line.split()
        if len(fields) < 14 or not device_pattern.match(fields[2]):
            continue
        try:
            result[fields[2]] = {
                "read_bytes": int(fields[5]) * 512,
                "write_bytes": int(fields[9]) * 512,
                "io_ms": int(fields[12]),
            }
        except ValueError:
            continue
    return result


def disk_rates(previous, current, elapsed):
    result = {}
    if elapsed <= 0.0:
        return result
    for device, counters in current.items():
        before = previous.get(device, counters)
        result[device] = {
            "read_kib_s": round(
                max(0, counters["read_bytes"] - before["read_bytes"])
                / 1024.0
                / elapsed,
                1,
            ),
            "write_kib_s": round(
                max(0, counters["write_bytes"] - before["write_bytes"])
                / 1024.0
                / elapsed,
                1,
            ),
            "busy_pct": round(
                min(
                    100.0,
                    max(0, counters["io_ms"] - before["io_ms"])
                    / (elapsed * 10.0),
                ),
                1,
            ),
        }
    return result


def network_counters():
    result = {}
    for interface in Path("/sys/class/net").glob("*"):
        statistics = interface / "statistics"
        values = {}
        for field in (
            "rx_bytes",
            "tx_bytes",
            "rx_packets",
            "tx_packets",
            "rx_errors",
            "tx_errors",
            "rx_dropped",
            "tx_dropped",
        ):
            try:
                values[field] = int(
                    (statistics / field).read_text().strip()
                )
            except (OSError, ValueError):
                values[field] = 0
        result[interface.name] = values
    return result


def network_rates(previous, current, elapsed):
    result = {}
    if elapsed <= 0.0:
        return result
    for interface, counters in current.items():
        before = previous.get(interface, counters)
        result[interface] = {
            "rx_kib_s": round(
                max(0, counters["rx_bytes"] - before["rx_bytes"])
                / 1024.0
                / elapsed,
                1,
            ),
            "tx_kib_s": round(
                max(0, counters["tx_bytes"] - before["tx_bytes"])
                / 1024.0
                / elapsed,
                1,
            ),
            "rx_packets_s": round(
                max(0, counters["rx_packets"] - before["rx_packets"])
                / elapsed,
                1,
            ),
            "tx_packets_s": round(
                max(0, counters["tx_packets"] - before["tx_packets"])
                / elapsed,
                1,
            ),
            "rx_errors_delta": max(
                0,
                counters["rx_errors"] - before["rx_errors"],
            ),
            "tx_errors_delta": max(
                0,
                counters["tx_errors"] - before["tx_errors"],
            ),
            "rx_dropped_delta": max(
                0,
                counters["rx_dropped"] - before["rx_dropped"],
            ),
            "tx_dropped_delta": max(
                0,
                counters["tx_dropped"] - before["tx_dropped"],
            ),
        }
    return result


def udp_counters():
    lines = read_text("/proc/net/snmp").splitlines()
    for index in range(len(lines) - 1):
        if not lines[index].startswith("Udp:"):
            continue
        if not lines[index + 1].startswith("Udp:"):
            continue
        names = lines[index].split()[1:]
        values = lines[index + 1].split()[1:]
        try:
            return {
                name: int(value)
                for name, value in zip(names, values)
            }
        except ValueError:
            return {}
    return {}


def counter_deltas(previous, current):
    return {
        key: max(0, value - previous.get(key, value))
        for key, value in current.items()
    }


def process_snapshot():
    ticks_by_label = {label: 0 for label in PROCESS_PATTERNS}
    rss_by_label = {label: 0 for label in PROCESS_PATTERNS}
    pids_by_label = {label: [] for label in PROCESS_PATTERNS}
    details_by_label = {
        label: {
            "threads": 0,
            "last_cpus": set(),
            "thread_last_cpu_counts": {},
            "cpu_allowed_lists": set(),
            "voluntary_context_switches": 0,
            "nonvoluntary_context_switches": 0,
        }
        for label in PROCESS_PATTERNS
    }

    for process_dir in Path("/proc").glob("[0-9]*"):
        try:
            cmdline = (process_dir / "cmdline").read_bytes()
            command = cmdline.replace(b"\0", b" ").decode(
                "utf-8",
                errors="replace",
            )
            if not command:
                continue
            comm = (process_dir / "comm").read_text().strip()
            # Launcher shells contain the full generated patrol command, which
            # mentions every process name and would otherwise be counted once
            # under every label.
            if comm in ("bash", "sh", "dash", "ssh", "timeout"):
                continue
            stat_fields = (process_dir / "stat").read_text().split()
            status = (process_dir / "status").read_text()
            ticks = int(stat_fields[13]) + int(stat_fields[14])
            last_cpu = int(stat_fields[38])
            rss_match = re.search(r"^VmRSS:\s+(\d+)", status, re.MULTILINE)
            rss_kib = int(rss_match.group(1)) if rss_match else 0
            threads_match = re.search(
                r"^Threads:\s+(\d+)",
                status,
                re.MULTILINE,
            )
            allowed_match = re.search(
                r"^Cpus_allowed_list:\s+(.+)$",
                status,
                re.MULTILINE,
            )
            voluntary_match = re.search(
                r"^voluntary_ctxt_switches:\s+(\d+)",
                status,
                re.MULTILINE,
            )
            nonvoluntary_match = re.search(
                r"^nonvoluntary_ctxt_switches:\s+(\d+)",
                status,
                re.MULTILINE,
            )
            pid = int(process_dir.name)
        except (OSError, ValueError, IndexError):
            continue

        task_last_cpu_counts = {}
        for task_dir in (process_dir / "task").glob("[0-9]*"):
            try:
                task_stat = (task_dir / "stat").read_text().split()
                task_last_cpu = int(task_stat[38])
            except (OSError, ValueError, IndexError):
                continue
            task_last_cpu_counts[task_last_cpu] = (
                task_last_cpu_counts.get(task_last_cpu, 0) + 1
            )
        if not task_last_cpu_counts:
            task_last_cpu_counts[last_cpu] = 1

        for label, pattern in PROCESS_PATTERNS.items():
            if pattern.search(command):
                ticks_by_label[label] += ticks
                rss_by_label[label] += rss_kib
                pids_by_label[label].append(pid)
                detail = details_by_label[label]
                detail["threads"] += (
                    int(threads_match.group(1))
                    if threads_match
                    else 0
                )
                detail["last_cpus"].update(
                    task_last_cpu_counts
                )
                for cpu_id, thread_count in (
                    task_last_cpu_counts.items()
                ):
                    detail["thread_last_cpu_counts"][cpu_id] = (
                        detail["thread_last_cpu_counts"].get(
                            cpu_id,
                            0,
                        )
                        + thread_count
                    )
                if allowed_match:
                    detail["cpu_allowed_lists"].add(
                        allowed_match.group(1).strip()
                    )
                detail["voluntary_context_switches"] += (
                    int(voluntary_match.group(1))
                    if voluntary_match
                    else 0
                )
                detail["nonvoluntary_context_switches"] += (
                    int(nonvoluntary_match.group(1))
                    if nonvoluntary_match
                    else 0
                )

    serializable_details = {}
    for label, detail in details_by_label.items():
        if not pids_by_label[label]:
            continue
        serializable_details[label] = {
            "threads": detail["threads"],
            "last_cpus": sorted(detail["last_cpus"]),
            "thread_last_cpu_counts": {
                str(cpu_id): count
                for cpu_id, count in sorted(
                    detail["thread_last_cpu_counts"].items()
                )
            },
            "cpu_allowed_lists": sorted(
                detail["cpu_allowed_lists"]
            ),
            "voluntary_context_switches":
                detail["voluntary_context_switches"],
            "nonvoluntary_context_switches":
                detail["nonvoluntary_context_switches"],
        }

    return (
        ticks_by_label,
        rss_by_label,
        pids_by_label,
        serializable_details,
    )


def cpu_percentages(previous, current, elapsed, ticks_per_second):
    if elapsed <= 0.0:
        return {}
    result = {}
    for label, ticks in current.items():
        delta = max(0, ticks - previous.get(label, ticks))
        if delta:
            result[label] = round(
                100.0 * delta / ticks_per_second / elapsed,
                1,
            )
    return result


def iso_time():
    return datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument(
        "--filesystem-path",
        default="/home/unitree/go2_fastlio_ws/patrol_logs",
    )
    parser.add_argument(
        "--output",
        default="",
        help="optional JSONL/text output; stdout remains the default",
    )
    parser.add_argument("--rotate-bytes", type=int, default=67108864)
    parser.add_argument("--rotate-files", type=int, default=8)
    return parser.parse_args()


def main():
    args = parse_args()
    interval = max(0.25, float(args.interval))
    output = None
    if args.output:
        output = RotatingTextOutput(
            args.output,
            args.rotate_bytes,
            args.rotate_files,
        )

    def emit(prefix, payload):
        line = prefix + " " + json.dumps(payload, sort_keys=True)
        if output is None:
            print(line, flush=True)
        else:
            output.write_line(line)

    ticks_per_second = float(os.sysconf("SC_CLK_TCK"))
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    previous_mono = time.monotonic()
    previous_cpu = read_cpu_totals()
    previous_per_cpu = read_per_cpu_counters()
    previous_ticks, _, _, _ = process_snapshot()
    previous_disk = disk_counters()
    previous_network = network_counters()
    previous_udp = udp_counters()
    deadline = previous_mono + interval

    emit(
        "PERF_MONITOR_START",
        {
            "interval_s": interval,
            "nvpmodel_status": read_text(
                "/var/lib/nvpmodel/status",
                "unknown",
            ),
            "kernel": read_text("/proc/sys/kernel/osrelease", "unknown"),
            "filesystem": filesystem_snapshot(args.filesystem_path),
            "wall_time": iso_time(),
        },
    )

    while not stopping:
        delay = deadline - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
        now_mono = time.monotonic()
        elapsed = max(1e-6, now_mono - previous_mono)
        wake_late_ms = max(0.0, (now_mono - deadline) * 1000.0)

        cpu = read_cpu_totals()
        per_cpu = read_per_cpu_counters()
        ticks, rss, pids, process_details = process_snapshot()
        disk = disk_counters()
        network = network_counters()
        udp = udp_counters()
        cpu_delta = max(0, cpu[0] - previous_cpu[0])
        idle_delta = max(0, cpu[1] - previous_cpu[1])
        system_cpu = (
            100.0 * (cpu_delta - idle_delta) / cpu_delta
            if cpu_delta > 0
            else 0.0
        )
        process_cpu = cpu_percentages(
            previous_ticks,
            ticks,
            elapsed,
            ticks_per_second,
        )
        cpu_ids = online_cpu_ids()
        temperatures = thermal_zones()
        try:
            load1, load5, load15 = os.getloadavg()
        except OSError:
            load1, load5, load15 = 0.0, 0.0, 0.0

        record = {
            "wall_time": iso_time(),
            "monotonic_s": round(now_mono, 3),
            "sample_elapsed_ms": round(elapsed * 1000.0, 3),
            "monitor_wake_late_ms": round(wake_late_ms, 3),
            "system_cpu_pct": round(system_cpu, 1),
            "per_cpu_pct": per_cpu_percentages(
                previous_per_cpu,
                per_cpu,
            ),
            "online_cpus": cpu_ids,
            "cpu_frequency": cpu_frequencies(cpu_ids),
            "accelerator": accelerator_snapshot(),
            "hwmon": hwmon_snapshot(),
            "load": [
                round(load1, 2),
                round(load5, 2),
                round(load15, 2),
            ],
            "procs_running": cpu[3],
            "procs_blocked": cpu[4],
            "context_switches_per_s": round(
                max(0, cpu[2] - previous_cpu[2]) / elapsed,
                1,
            ),
            "pressure_cpu": read_pressure("cpu"),
            "pressure_memory": read_pressure("memory"),
            "pressure_io": read_pressure("io"),
            "memory": read_memory(),
            "thermal_zones_c": temperatures,
            "max_temperature_c": (
                max(temperatures.values())
                if temperatures
                else None
            ),
            "filesystem": filesystem_snapshot(args.filesystem_path),
            "disk_io": disk_rates(previous_disk, disk, elapsed),
            "network": network_rates(
                previous_network,
                network,
                elapsed,
            ),
            "udp_deltas": counter_deltas(previous_udp, udp),
            "process_cpu_pct": process_cpu,
            "process_rss_mib": {
                label: round(value / 1024.0, 1)
                for label, value in rss.items()
                if value
            },
            "process_pids": {
                label: values
                for label, values in pids.items()
                if values
            },
            "process_scheduling": process_details,
        }
        emit("PERF_SAMPLE", record)

        previous_mono = now_mono
        previous_cpu = cpu
        previous_per_cpu = per_cpu
        previous_ticks = ticks
        previous_disk = disk
        previous_network = network
        previous_udp = udp
        while deadline <= now_mono:
            deadline += interval

    emit("PERF_MONITOR_STOP", {"wall_time": iso_time()})
    if output is not None:
        output.close()


if __name__ == "__main__":
    main()
