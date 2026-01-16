#!/usr/bin/env python3
"""
narrator.py (dashboard-friendly)

- Subscribes:
  - /servoing/status (std_msgs/String)                  [configurable]
  - /rov/servo_mode_active (std_msgs/Bool)             [configurable]
  - /mavros/global_position/rel_alt (std_msgs/Float64) [configurable]
  - /joy (sensor_msgs/Joy)                             [optional]

- Publishes:
  - /rov/mission_narrator (std_msgs/String)
  - /rov/mission_log (std_msgs/String)
  - /rov/mission_summary (std_msgs/String)
  - /rov/mission_summary_json (std_msgs/String)

- Services:
  - /mission_narrator/start_run (std_srvs/Trigger)
  - /mission_narrator/stop_run (std_srvs/Trigger)

Adds:
- More human-like narration, including depth phrased as "at about X m depth".
- Final report includes a "Human timeline" with the same style + depth per event.
- Defensive shutdown to avoid "rcl_shutdown already called" on Ctrl+C.
"""

import re
import time
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


@dataclass
class Event:
    t_wall: float
    t_str: str
    depth: Optional[float]
    status: str
    state_key: str


class MissionNarrator(Node):
    def __init__(self):
        super().__init__("mission_narrator")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("topic_status", "/servoing/status")
        self.declare_parameter("topic_auto", "/rov/servo_mode_active")
        self.declare_parameter("topic_depth", "/mavros/global_position/rel_alt")

        self.declare_parameter("use_joy", True)
        self.declare_parameter("topic_joy", "/joy")
        self.declare_parameter("lunge_button_idx", 0)

        self.declare_parameter("pub_narrator", "/rov/mission_narrator")
        self.declare_parameter("pub_log", "/rov/mission_log")
        self.declare_parameter("pub_summary", "/rov/mission_summary")
        self.declare_parameter("pub_summary_json", "/rov/mission_summary_json")

        self.declare_parameter("update_interval_sec", 4.0)
        self.declare_parameter("log_publish_interval_sec", 0.5)
        self.declare_parameter("log_keep_lines", 200)
        self.declare_parameter("lost_timeout_sec", 3.0)

        self.declare_parameter("save_to_disk", True)
        self.declare_parameter("report_dir", str(Path.home() / "bluerov_mission_reports"))
        self.declare_parameter("file_prefix", "mission")

        # read them
        self.topic_status = str(self.get_parameter("topic_status").value)
        self.topic_auto = str(self.get_parameter("topic_auto").value)
        self.topic_depth = str(self.get_parameter("topic_depth").value)

        self.use_joy = bool(self.get_parameter("use_joy").value)
        self.topic_joy = str(self.get_parameter("topic_joy").value)
        self.lunge_button_idx = int(self.get_parameter("lunge_button_idx").value)

        self.pub_narrator_topic = str(self.get_parameter("pub_narrator").value)
        self.pub_log_topic = str(self.get_parameter("pub_log").value)
        self.pub_summary_topic = str(self.get_parameter("pub_summary").value)
        self.pub_summary_json_topic = str(self.get_parameter("pub_summary_json").value)

        self.update_interval_sec = float(self.get_parameter("update_interval_sec").value)
        self.log_publish_interval_sec = float(self.get_parameter("log_publish_interval_sec").value)
        self.log_keep_lines = int(self.get_parameter("log_keep_lines").value)
        self.lost_timeout_sec = float(self.get_parameter("lost_timeout_sec").value)

        self.save_to_disk = bool(self.get_parameter("save_to_disk").value)
        self.report_dir = Path(str(self.get_parameter("report_dir").value)).expanduser()
        self.file_prefix = str(self.get_parameter("file_prefix").value)

        # -------------------------
        # Runtime state
        # -------------------------
        self.auto_active = False
        self.depth: Optional[float] = None

        self.status_text = "IDLE"
        self.last_status_text: Optional[str] = None

        self.last_narration_time = 0.0
        self.last_log_publish_time = 0.0

        self.summary_emitted = False
        self.run_id: Optional[str] = None
        self.mission_start_wall: Optional[float] = None
        self.mission_end_wall: Optional[float] = None
        self.end_reason: str = "unknown"

        # Timing per state
        self.last_state_key: Optional[str] = None
        self.last_state_change_wall: Optional[float] = None
        self.time_in_state: Dict[str, float] = {}

        # Events + rolling log
        self.events: List[Event] = []
        self.log_lines: List[str] = []
        self.status_change_count = 0

        # Lost/recovery counters
        self.lost_since_wall: Optional[float] = None
        self.last_lost_notice_wall = 0.0
        self.recovery_count = 0
        self.lost_event_count = 0

        # Depth stats
        self.min_depth: Optional[float] = None
        self.max_depth: Optional[float] = None

        # Milestones
        self.first_locked_wall: Optional[float] = None
        self.first_lunge_wall: Optional[float] = None

        # Manual lunge tracking
        self.joy_lunge_pressed = False
        self.manual_lunge_active = False
        self.manual_lunge_start_wall: Optional[float] = None
        self.manual_lunge_total_sec = 0.0
        self.manual_lunge_count = 0

        # File paths
        self.log_path: Optional[Path] = None
        self.summary_path: Optional[Path] = None

        # -------------------------
        # Publishers
        # -------------------------
        self.pub_narrator = self.create_publisher(String, self.pub_narrator_topic, 10)
        self.pub_log = self.create_publisher(String, self.pub_log_topic, 10)
        self.pub_summary = self.create_publisher(String, self.pub_summary_topic, 10)
        self.pub_summary_json = self.create_publisher(String, self.pub_summary_json_topic, 10)

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(String, self.topic_status, self.status_callback, 10)
        self.create_subscription(Bool, self.topic_auto, self.auto_callback, 10)
        self.create_subscription(Float64, self.topic_depth, self.depth_callback, 10)

        if self.use_joy:
            self.create_subscription(Joy, self.topic_joy, self.joy_callback, 10)

        # -------------------------
        # Services
        # -------------------------
        self.srv_start = self.create_service(Trigger, "/mission_narrator/start_run", self._srv_start_run)
        self.srv_stop = self.create_service(Trigger, "/mission_narrator/stop_run", self._srv_stop_run)

        # -------------------------
        # Timer
        # -------------------------
        self.timer = self.create_timer(0.2, self.timer_callback)

        self._log_event("Narrator started and ready.")
        self.get_logger().info("MissionNarrator started.")

    # -------------------------
    # Utilities
    # -------------------------
    def _now_wall(self) -> float:
        return time.time()

    def _now_str(self) -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _ensure_files(self):
        if not self.save_to_disk or self.run_id is None:
            return

        try:
            self.report_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.get_logger().warning(f"Could not create report directory {self.report_dir}: {e}")

        if self.log_path is None:
            self.log_path = self.report_dir / f"{self.file_prefix}_{self.run_id}.log"
        if self.summary_path is None:
            self.summary_path = self.report_dir / f"{self.file_prefix}_{self.run_id}_summary.txt"

    def _append_file_line(self, path: Path, line: str):
        try:
            with path.open("a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception as e:
            self.get_logger().warning(f"Failed writing to {path}: {e}")

    def _log_event(self, text: str):
        line = f"[{self._now_str()}] {text}"
        self.log_lines.append(line)

        if len(self.log_lines) > self.log_keep_lines:
            self.log_lines = self.log_lines[-self.log_keep_lines :]

        self._ensure_files()
        if self.save_to_disk and self.log_path is not None:
            self._append_file_line(self.log_path, line)

        self.get_logger().info(line)

    def _publish_narration(self, text: str):
        self.pub_narrator.publish(String(data=text))
        self.last_narration_time = self._now_wall()

    def _publish_rolling_log(self):
        self.pub_log.publish(String(data="\n".join(self.log_lines)))
        self.last_log_publish_time = self._now_wall()

    # -------------------------
    # State normalization + parsing
    # -------------------------
    def _normalize_state(self, status_text: str) -> str:
        if not status_text:
            return "UNKNOWN"

        s = status_text.strip().upper()

        if "FOUND HANDLE" in s:
            return "SQUARING"
        if "STABILIZING" in s or "HOLDING" in s or s.startswith("GRACE:") or "GRACE PERIOD" in s:
            return "LUNGE_READY"
        if "MANUAL LUNGE" in s:
            return "LUNGING"
        if "HOLD 'A' TO LUNGE" in s or "HOLD A TO LUNGE" in s:
            return "LUNGE_READY"
        if s.startswith("MANUAL"):
            return "MANUAL"
        if s.startswith("SEARCH:"):
            return "SEARCH_HANDLE"
        if "ORBITING" in s or "LOCKED" in s:
            return "SQUARING"
        if "RECOVERING" in s or "RECOVERY" in s:
            return "RECOVERY_BACKOFF"
        if "LOST" in s:
            return "LOST"
        if "LUNGE_READY" in s:
            return "LUNGE_READY"
        if "LUNGING" in s:
            return "LUNGING"
        if "COMPLETE" in s:
            return "COMPLETE"
        if "DIVING" in s:
            return "DIVING"

        token = re.split(r"[\s(]", s, maxsplit=1)[0]
        return token if token else "UNKNOWN"

    def _update_time_in_state(self, new_state_key: str, now_wall: float):
        if self.last_state_key is None:
            self.last_state_key = new_state_key
            self.last_state_change_wall = now_wall
            return

        if self.last_state_change_wall is None:
            self.last_state_change_wall = now_wall
            self.last_state_key = new_state_key
            return

        if new_state_key == self.last_state_key:
            return

        dt = now_wall - self.last_state_change_wall
        self.time_in_state[self.last_state_key] = self.time_in_state.get(self.last_state_key, 0.0) + max(0.0, dt)

        self.last_state_key = new_state_key
        self.last_state_change_wall = now_wall

    def _fmt_duration(self, seconds: float) -> str:
        seconds = max(0.0, float(seconds))
        m = int(seconds // 60)
        s = seconds - 60 * m
        return f"{m}m {s:.1f}s" if m > 0 else f"{s:.1f}s"

    # -------------------------
    # Human phrasing
    # -------------------------
    def _depth_phrase(self, depth_value: Optional[float]) -> str:
        if depth_value is None:
            return ""
        # Keep your previous convention (abs rel_alt -> "depth")
        d = abs(float(depth_value))
        return f" at about {d:.2f} m depth"

    def _human_sentence(self, state_key: str, status_text: str, depth_value: Optional[float], tense: str) -> str:
        """
        tense: 'present' or 'past'
        """
        up = (status_text or "").upper()
        depth = self._depth_phrase(depth_value)

        def v(present: str, past: str) -> str:
            return present if tense == "present" else past

        if state_key == "MANUAL":
            return v(f"I'm in manual mode and standing by{depth}.",
                     f"I was in manual mode and standing by{depth}.")

        if state_key == "DIVING":
            return v(f"I'm diving to mission depth and scanning for the target{depth}.",
                     f"I was diving to mission depth and scanning for the target{depth}.")

        if state_key == "SEARCH_HANDLE":
            if "CRAWLING" in up:
                return v(f"I see the target but not the handle yet—I'm crawling forward to improve the view{depth}.",
                         f"I saw the target but not the handle yet, so I crawled forward to improve the view{depth}.")
            if "LOST BOX" in up or "LOST" in up:
                return v(f"I lost the target—I'm holding position and searching again{depth}.",
                         f"I lost the target and held position while searching again{depth}.")
            return v(f"I'm searching for the handle while holding depth{depth}.",
                     f"I was searching for the handle while holding depth{depth}.")

        if state_key == "SQUARING":
            if "ORBITING" in up:
                return v(f"I'm orbiting to reduce parallax and square up with the handle{depth}.",
                         f"I orbited to reduce parallax and square up with the handle{depth}.")
            if "LOCKED" in up:
                return v(f"I'm locked on and closing distance while keeping alignment stable{depth}.",
                         f"I was locked on and closing distance while keeping alignment stable{depth}.")
            return v(f"I'm squaring up—adjusting yaw and lateral position{depth}.",
                     f"I was squaring up by adjusting yaw and lateral position{depth}.")

        if state_key == "LUNGE_READY":
            live_button = self.use_joy and (self.joy_lunge_pressed or self.manual_lunge_active)

            if tense == "present" and live_button:
                return f"I'm pushing forward while you hold the lunge button{depth}."
            return v(f"I'm lined up and ready—hold the lunge button to drive forward{depth}.",
                     f"I was lined up and ready to lunge{depth}.")

        if state_key == "LUNGING":
            # The exact style you asked for, but in present/past tense.
            return v(f"I'm lunging at it now{depth}.",
                     f"I was lunging at it{depth}.")

        if state_key == "RECOVERY_BACKOFF":
            return v(f"I lost reliable detections—I'm backing off to recover the target{depth}.",
                     f"I lost reliable detections and backed off to recover the target{depth}.")

        if state_key == "LOST":
            return v(f"I can't see the target right now—I'm re-scanning{depth}.",
                     f"I couldn't see the target for a while and kept re-scanning{depth}.")

        if state_key == "COMPLETE":
            return v(f"I'm done—mission complete. I'm stopped and waiting for operator input{depth}.",
                     f"The mission completed and I stopped, waiting for operator input{depth}.")

        return v(f"I'm currently in: {status_text}{depth}.",
                 f"I was in: {status_text}{depth}.")

    def _build_narration(self, state_key: str, status_text: str) -> str:
        return self._human_sentence(state_key, status_text, self.depth, tense="present")

    # -------------------------
    # ROS callbacks
    # -------------------------
    def auto_callback(self, msg: Bool):
        prev = self.auto_active
        self.auto_active = bool(msg.data)

        if self.auto_active and not prev:
            self._start_run_auto()
            return

        if (not self.auto_active) and prev:
            self._log_event("Autonomy disabled by operator.")
            if not self.summary_emitted and self.mission_start_wall is not None:
                self.end_reason = "operator_disabled"
                self._emit_summary()
            self._publish_narration("Autonomy disengaged. Operator has control.")

    def depth_callback(self, msg: Float64):
        self.depth = float(msg.data)

        if self.min_depth is None or self.depth < self.min_depth:
            self.min_depth = self.depth
        if self.max_depth is None or self.depth > self.max_depth:
            self.max_depth = self.depth

    def status_callback(self, msg: String):
        self.status_text = msg.data

    def joy_callback(self, msg: Joy):
        try:
            if len(msg.buttons) > self.lunge_button_idx:
                self.joy_lunge_pressed = bool(msg.buttons[self.lunge_button_idx])
        except Exception:
            pass

    # -------------------------
    # Manual-run helpers (start/stop)
    # -------------------------
    def _start_run_auto(self):
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_utc")
        self.mission_start_wall = self._now_wall()
        self.mission_end_wall = None
        self.end_reason = "running"
        self.summary_emitted = False

        self.events.clear()
        self.log_lines.clear()
        self.time_in_state.clear()

        self.last_state_key = None
        self.last_state_change_wall = None

        self.status_change_count = 0
        self.recovery_count = 0
        self.lost_event_count = 0
        self.lost_since_wall = None
        self.last_lost_notice_wall = 0.0

        self.min_depth = None
        self.max_depth = None

        self.first_locked_wall = None
        self.first_lunge_wall = None

        self.manual_lunge_active = False
        self.manual_lunge_start_wall = None
        self.manual_lunge_total_sec = 0.0
        self.manual_lunge_count = 0

        self.log_path = None
        self.summary_path = None

        self._ensure_files()

        self._log_event(f"Autonomy enabled. Run ID: {self.run_id}")
        self._publish_narration("Autonomy engaged. Starting the visual-servoing mission.")

    def _start_run_manual(self):
        self._start_run_auto()
        self._publish_narration("Mission run started by dashboard (manual start).")

    def _stop_run_manual(self):
        if not self.summary_emitted and self.mission_start_wall is not None:
            self.end_reason = "dashboard_stop"
            self._emit_summary()
        self._publish_narration("Mission run stopped by dashboard (manual stop).")

    # -------------------------
    # Service callbacks
    # -------------------------
    def _srv_start_run(self, request, response):
        try:
            self._start_run_manual()
            response.success = True
            response.message = f"Started run {self.run_id}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to start run: {e}"
        return response

    def _srv_stop_run(self, request, response):
        try:
            self._stop_run_manual()
            response.success = True
            response.message = f"Stopped run {self.run_id}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop run: {e}"
        return response

    # -------------------------
    # Manual lunge tracking
    # -------------------------
    def _manual_lunge_enter(self, now: float):
        if self.manual_lunge_active:
            return

        self.manual_lunge_active = True
        self.manual_lunge_start_wall = now
        self.manual_lunge_count += 1

        if self.first_lunge_wall is None:
            self.first_lunge_wall = now

        self._log_event(f"Manual lunge started (count={self.manual_lunge_count}).")

    def _manual_lunge_exit(self, now: float):
        if not self.manual_lunge_active:
            return

        self.manual_lunge_active = False

        if self.manual_lunge_start_wall is not None:
            dt = max(0.0, now - self.manual_lunge_start_wall)
            self.manual_lunge_total_sec += dt
            self._log_event(
                f"Manual lunge ended (duration={dt:.2f}s, total={self.manual_lunge_total_sec:.2f}s)."
            )
            self.manual_lunge_start_wall = None

    # -------------------------
    # Summary builders
    # -------------------------
    def _build_detailed_summary(self) -> str:
        start = self.mission_start_wall
        end = self.mission_end_wall if self.mission_end_wall is not None else self._now_wall()
        dur = self._fmt_duration(end - start) if start is not None else "unknown"

        def rel(t):
            if t is None or start is None:
                return "n/a"
            return self._fmt_duration(t - start)

        lines: List[str] = []
        lines.append("Mission report")
        lines.append(f"- Run ID: {self.run_id}")
        lines.append(f"- End reason: {self.end_reason}")
        lines.append(f"- Start (local): {datetime.fromtimestamp(start).strftime('%Y-%m-%d %H:%M:%S') if start else 'unknown'}")
        lines.append(f"- End (local): {datetime.fromtimestamp(end).strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"- Duration: {dur}")
        lines.append(f"- Status changes: {self.status_change_count}")
        lines.append(f"- Lost events: {self.lost_event_count}")
        lines.append(f"- Recovery events: {self.recovery_count}")

        if self.min_depth is not None and self.max_depth is not None:
            lines.append(f"- Depth range (rel_alt): min {self.min_depth:.2f} m, max {self.max_depth:.2f} m")
        if self.depth is not None:
            lines.append(f"- Final depth (rel_alt): {self.depth:.2f} m")

        lines.append(f"- First locked: {rel(self.first_locked_wall)}")
        lines.append(f"- First lunge: {rel(self.first_lunge_wall)}")
        lines.append(f"- Manual lunge count: {self.manual_lunge_count}")
        lines.append(f"- Manual lunge total time: {self.manual_lunge_total_sec:.2f} s")

        if self.time_in_state:
            lines.append("")
            lines.append("Time in states (approx):")
            for k, v in sorted(self.time_in_state.items(), key=lambda kv: kv[1], reverse=True):
                lines.append(f"- {k}: {self._fmt_duration(v)}")

        # Human-like timeline with depth per event
        lines.append("")
        lines.append("Human timeline (last 60 events):")
        for ev in self.events[-60:]:
            human = self._human_sentence(ev.state_key, ev.status, ev.depth, tense="past")
            lines.append(f"- {ev.t_str}: {human}")

        # Keep raw status timeline for debugging
        lines.append("")
        lines.append("Raw timeline (last 60 status events):")
        for ev in self.events[-60:]:
            d = f"{ev.depth:.2f}m" if ev.depth is not None else "n/a"
            lines.append(f"- {ev.t_str} | depth {d} | {ev.status}")

        if self.save_to_disk:
            self._ensure_files()
            if self.log_path is not None:
                lines.append("")
                lines.append(f"Saved rolling log: {self.log_path}")
            if self.summary_path is not None:
                lines.append(f"Saved summary: {self.summary_path}")

        return "\n".join(lines)

    def _emit_summary(self):
        if self.summary_emitted:
            return

        self.summary_emitted = True
        self.mission_end_wall = self._now_wall()

        # close manual lunge if active
        self._manual_lunge_exit(self.mission_end_wall)

        # account last state's remaining time
        if self.last_state_key is not None and self.last_state_change_wall is not None:
            dt = self.mission_end_wall - self.last_state_change_wall
            self.time_in_state[self.last_state_key] = self.time_in_state.get(self.last_state_key, 0.0) + max(0.0, dt)

        summary = self._build_detailed_summary()

        # publish textual summary
        self.pub_summary.publish(String(data=summary))

        # publish machine-friendly JSON summary
        json_obj = {
            "run_id": self.run_id,
            "end_reason": self.end_reason,
            "start_ts": int(self.mission_start_wall) if self.mission_start_wall else None,
            "end_ts": int(self.mission_end_wall) if self.mission_end_wall else None,
            "duration_s": (self.mission_end_wall - self.mission_start_wall)
            if (self.mission_end_wall and self.mission_start_wall)
            else None,
            "log_path": str(self.log_path) if self.log_path else None,
            "summary_path": str(self.summary_path) if self.summary_path else None,
            "status_changes": self.status_change_count,
            "lost_events": self.lost_event_count,
            "recovery_events": self.recovery_count,
        }
        try:
            self.pub_summary_json.publish(String(data=json.dumps(json_obj)))
        except Exception:
            pass

        # write to disk
        self._ensure_files()
        if self.save_to_disk and self.summary_path is not None:
            try:
                self.summary_path.write_text(summary + "\n", encoding="utf-8")
                self._log_event(f"Saved summary: {self.summary_path}")
            except Exception as e:
                self.get_logger().warning(f"Failed writing summary: {e}")

        # explicitly notify dashboard where the files are
        if self.save_to_disk and self.summary_path is not None:
            self._publish_narration(f"Mission summary saved to {self.summary_path}")
        else:
            self._publish_narration("Mission summary emitted (not saved to disk).")

        self._log_event("Mission summary emitted.")

    # -------------------------
    # Timer
    # -------------------------
    def timer_callback(self):
        now = self._now_wall()

        # Always publish rolling log periodically for UI
        if now - self.last_log_publish_time >= self.log_publish_interval_sec:
            self._publish_rolling_log()

        # Track manual lunge based on status text even if joy isn't used
        status_up = (self.status_text or "").upper()
        if "MANUAL LUNGE" in status_up or (self.use_joy and self.joy_lunge_pressed):
            self._manual_lunge_enter(now)
        else:
            self._manual_lunge_exit(now)

        # If auto not on, narrator stays idle
        if not self.auto_active:
            return

        # Status change logic
        if self.status_text != self.last_status_text:
            self.status_change_count += 1

            state_key = self._normalize_state(self.status_text)
            self._update_time_in_state(state_key, now)

            # Snapshot event (depth at the moment)
            self.events.append(
                Event(
                    t_wall=now,
                    t_str=self._now_str(),
                    depth=self.depth,
                    status=self.status_text,
                    state_key=state_key,
                )
            )

            self._log_event(f"Status: {self.status_text}")

            if state_key == "RECOVERY_BACKOFF":
                self.recovery_count += 1
            if "LOST" in status_up:
                self.lost_event_count += 1
            if "LOCKED" in status_up and self.first_locked_wall is None:
                self.first_locked_wall = now
            if state_key == "LUNGING" and self.first_lunge_wall is None:
                self.first_lunge_wall = now

            # Immediate narration
            try:
                self._publish_narration(self._build_narration(state_key, self.status_text))
            except Exception:
                pass

            # If the servoing publishes COMPLETE, handle it
            if state_key == "COMPLETE" and not self.summary_emitted:
                self.end_reason = "servo_complete"
                self._emit_summary()

            self.last_status_text = self.status_text

        # Prolonged lost tracking
        is_lost_now = ("LOST" in status_up)
        if is_lost_now:
            if self.lost_since_wall is None:
                self.lost_since_wall = now
            if (now - self.lost_since_wall) >= self.lost_timeout_sec and (now - self.last_lost_notice_wall) >= self.lost_timeout_sec:
                self.last_lost_notice_wall = now
                self._log_event("Targets have been lost for several seconds; recovery likely.")
        else:
            self.lost_since_wall = None

        # Periodic narration refresh
        if now - self.last_narration_time >= self.update_interval_sec:
            state_key = self._normalize_state(self.status_text)
            try:
                self._publish_narration(self._build_narration(state_key, self.status_text))
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionNarrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        # Guard against double-shutdown on Humble
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

