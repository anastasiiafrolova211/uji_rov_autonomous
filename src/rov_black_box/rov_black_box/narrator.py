#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Float64, Bool

import math
import time


class MissionNarrator(Node):
    """
    Cognitive narrative 'think-aloud' node for the BlueROV visual servoing mission.
    Subscribes to high-level mission state and basic telemetry, then publishes
    natural language explanations for the operator.
    """

    def __init__(self):
        super().__init__('mission_narrator')

        # --- Parameters ---
        self.declare_parameter('update_interval', 5.0)   # seconds between main narrative updates
        self.declare_parameter('handle_lost_timeout', 3.0)
        self.declare_parameter('min_conf_box', 0.75)     # threshold for initial box detection narrative
        self.declare_parameter('min_conf_handle', 0.60)  # threshold for stable handle narrative

        self.update_interval = float(self.get_parameter('update_interval').value)
        self.handle_lost_timeout = float(self.get_parameter('handle_lost_timeout').value)
        self.min_conf_box = float(self.get_parameter('min_conf_box').value)
        self.min_conf_handle = float(self.get_parameter('min_conf_handle').value)

        # --- Internal state ---
        self.mission_state = "IDLE"
        self.auto_active = False
        self.depth = 0.0

        # If you later publish these from autonomous.py, you can use them directly
        self.box_conf = None        # in [0,1]
        self.box_angle_deg = None   # relative heading
        self.handle_conf = None     # in [0,1]
        self.handle_distance_m = None
        self.handle_visible = False

        self.last_handle_seen_time = None
        self.position_index = 0
        self.last_update_time = 0.0
        self.summary_emitted = False

        # Remember a short history of key events for a final summary (attachment done)
        self.event_log = []

        # --- Publishers ---
        self.narrator_pub = self.create_publisher(String, '/rov/mission_narrator', 10)
        self.summary_pub = self.create_publisher(String, '/rov/mission_summary', 10)

        # --- Subscribers ---
        # 1) Mission state (string) – you publish this from autonomous.py
        self.create_subscription(
            String,
            '/rov/mission_state',
            self.mission_state_callback,
            10
        )

        # 2) Auto-mode state (Bool) – you already have /rov/servo_mode_active
        self.create_subscription(
            Bool,
            '/rov/servo_mode_active',
            self.auto_state_callback,
            10
        )

        # 3) Depth (Float64)
        self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.depth_callback,
            10
        )

        # 4) Optional detection info (define and publish later if needed)
        #    Example topic names; you can adapt these to your actual implementation.
        #    /rov/blackbox_confidence (Float64) 0–1
        #    /rov/blackbox_angle_deg (Float64)
        #    /rov/handle_confidence   (Float64)
        #    /rov/handle_distance_m   (Float64)
        #    /rov/handle_visible      (Bool)

        self.create_subscription(
            Float64,
            '/rov/blackbox_confidence',
            self.box_conf_callback,
            10
        )
        self.create_subscription(
            Float64,
            '/rov/blackbox_angle_deg',
            self.box_angle_callback,
            10
        )
        self.create_subscription(
            Float64,
            '/rov/handle_confidence',
            self.handle_conf_callback,
            10
        )
        self.create_subscription(
            Float64,
            '/rov/handle_distance_m',
            self.handle_distance_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/rov/handle_visible',
            self.handle_visible_callback,
            10
        )

        # --- Timer for periodic narration ---
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("MissionNarrator node started (cognitive narrative mode).")

    # ========================
    #  Callbacks
    # ========================

    def mission_state_callback(self, msg: String):
        prev = self.mission_state
        self.mission_state = msg.data

        # Log state transitions
        if self.mission_state != prev:
            self.event_log.append(f"State changed: {prev} -> {self.mission_state}")
            # Immediate short update for some key transitions
            if self.mission_state == "LUNGING":
                self.publish_line("I am executing the attachment lunge towards the handle.")
            elif self.mission_state == "COMPLETE":
                self.publish_line("Carabiner attachment sequence completed. Awaiting operator verification.")
                # Trigger final summary once
                if not self.summary_emitted:
                    self.emit_summary()

    def auto_state_callback(self, msg: Bool):
        self.auto_active = msg.data
        if self.auto_active:
            self.event_log.append("Autonomous mode engaged by teleoperator.")
        else:
            self.event_log.append("Autonomous mode disengaged; teleoperator in control.")

    def depth_callback(self, msg: Float64):
        self.depth = msg.data

    def box_conf_callback(self, msg: Float64):
        self.box_conf = float(msg.data)

    def box_angle_callback(self, msg: Float64):
        self.box_angle_deg = float(msg.data)

    def handle_conf_callback(self, msg: Float64):
        self.handle_conf = float(msg.data)
        if self.handle_conf is not None and self.handle_conf >= self.min_conf_handle:
            self.last_handle_seen_time = self.get_clock().now()
            self.handle_visible = True

    def handle_distance_callback(self, msg: Float64):
        self.handle_distance_m = float(msg.data)

    def handle_visible_callback(self, msg: Bool):
        self.handle_visible = msg.data
        if self.handle_visible:
            self.last_handle_seen_time = self.get_clock().now()

    # ========================
    #  Main timer
    # ========================

    def timer_callback(self):
        # Only narrate when autonomous is active
        if not self.auto_active:
            return

        now = time.time()
        if now - self.last_update_time < self.update_interval:
            # Short-circuit unless enough time passed for the next step in the think-aloud sequence
            return

        self.last_update_time = now
        self.position_index += 1

        text = self.build_narration(now)
        if text:
            self.publish_line(text)

    # ========================
    #  Narration logic
    # ========================

    def build_narration(self, now: float) -> str:
        """
        Implements the cognitive narrative rules you described.
        Generates a short natural-language explanation depending on
        mission_state and perception signals.
        """

        # 1. Initial Detection (DIVING + good box detection)
        if self.mission_state == "DIVING":
            if self.box_conf is not None and self.box_conf >= self.min_conf_box:
                # Determine relative direction text if angle available
                if self.box_angle_deg is not None:
                    if self.box_angle_deg < -5:
                        pos_text = "left of my current heading"
                    elif self.box_angle_deg > 5:
                        pos_text = "right of my current heading"
                    else:
                        pos_text = "roughly straight ahead"
                else:
                    pos_text = "within the camera field of view"

                conf_pct = int(self.box_conf * 100)
                depth_str = f"{abs(self.depth):.1f}m" if self.depth != 0.0 else "current depth"
                self.event_log.append(
                    f"Initial box detection at depth {depth_str}, confidence {conf_pct}%."
                )

                return (
                    f"I am now in position {self.position_index} where I started autonomous mode "
                    f"after teleoperation. The camera detects an object 'blackbox' with "
                    f"{conf_pct}% confidence. This meets the requirement for initiating autonomous mode "
                    f"by the teleoperator. The object is located {pos_text}."
                )
            else:
                return (
                    f"I am diving and searching for the black box. Maintaining depth around "
                    f"{abs(self.depth):.1f} meters while scanning the scene."
                )

        # 2. Approach Strategy (APPROACHING, no reliable handle yet)
        if self.mission_state == "APPROACHING":
            if not self.handle_visible or (self.handle_conf is None) or (self.handle_conf < self.min_conf_handle):
                # Approach black box calmly
                self.event_log.append("Approaching black box without reliable handle detection yet.")
                return (
                    "My goal is to approach the black box. I adjust my heading and surge forward at low speed "
                    "to keep the object stable in the camera view and improve identification."
                )
            else:
                # 3. Handle Detection (consistent handle detection)
                conf_pct = int(self.handle_conf * 100)
                dist_txt = ""
                if self.handle_distance_m is not None:
                    dist_txt = f" at approximately {self.handle_distance_m:.1f} meters away"
                self.event_log.append(
                    f"Handle detected with confidence {conf_pct}%, distance {self.handle_distance_m}m."
                )
                return (
                    f"The black box handle has been detected with consistent confidence above 60%, currently "
                    f"around {conf_pct}%{dist_txt}. I am updating my approach strategy to align with the handle "
                    f"to prepare for grasping."
                )

        # 4. Unexpected Scenario – handle is lost
        if self.mission_state in ["APPROACHING", "LUNGE_READY", "LUNGING"]:
            lost = False
            if self.last_handle_seen_time is None:
                lost = True
            else:
                # If we have not seen the handle for a while
                dt = (self.get_clock().now() - self.last_handle_seen_time).nanoseconds * 1e-9
                if dt > self.handle_lost_timeout:
                    lost = True

            if lost:
                self.event_log.append("Handle lost – backing up and circling to re-acquire.")
                return (
                    "Handle is lost! It is either no longer in view or the visibility has changed so I can no "
                    "longer reliably identify it. I will now back up until the black box is at a safer distance, "
                    "then slowly circle around it until I can find the handle again."
                )

        # 5. Lunge Ready (alignment good)
        if self.mission_state == "LUNGE_READY":
            self.event_log.append("Final alignment before lunge.")
            return (
                "Handle alignment and distance are now within the target range. I am holding position and "
                "finalizing alignment before executing the attachment lunge."
            )

        # 6. LUNGING is handled in mission_state_callback for immediate one-shot text
        #    Here we can optionally add periodic reinforcement, but usually one message is enough.

        # 7. COMPLETE is also handled in mission_state_callback; here we focus on summary.

        # Default fallback – short status
        return (
            f"Current mission state is {self.mission_state}. Maintaining stable posture while monitoring "
            "the black box and handle detections."
        )

    # ========================
    #  Publishing helpers
    # ========================

    def publish_line(self, text: str):
        msg = String()
        msg.data = text
        self.narrator_pub.publish(msg)
        self.get_logger().info(f"NARRATOR: {text}")

    def emit_summary(self):
        """
        Emit a concise operation summary once the attachment is complete.
        """
        self.summary_emitted = True

        # Very simple textual summary; you can expand this with more structure if desired.
        summary_lines = [
            "Mission summary:",
            "- Autonomous mode was engaged to perform black box carabiner attachment.",
        ]
        if self.event_log:
            summary_lines.append("- Key events during operation:")
            for e in self.event_log[-10:]:  # last 10 events to keep it short
                summary_lines.append(f"  • {e}")
        if self.handle_distance_m is not None:
            summary_lines.append(
                f"- Final handle distance before lunge was approximately {self.handle_distance_m:.2f}m."
            )
        summary_lines.append("- Carabiner attachment sequence has finished. Awaiting operator confirmation.")

        summary_msg = String()
        summary_msg.data = " ".join(summary_lines)
        self.summary_pub.publish(summary_msg)
        self.get_logger().info("MISSION SUMMARY EMITTED.")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNarrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

