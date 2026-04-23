#!/usr/bin/env python3
"""三机 Leader-Follower + 一致性补偿 + 1Hz状态互播 + 按航迹完成后自动降落。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry


@dataclass
class UAVState:
    pos: np.ndarray
    vel: np.ndarray
    stamp_sec: float


class FormationCoordinator(Node):
    def __init__(self):
        super().__init__("formation_coordinator")

        self.uav_ids = [1, 2, 3]
        self.leader_id = 1
        self.follow_offsets = {2: np.array([0.0, -4.0, 0.0]), 3: np.array([0.0, 4.0, 0.0])}

        # 图论邻接矩阵（全连通）
        self.A = np.array([[0, 1, 1], [1, 0, 1], [1, 1, 0]], dtype=float)
        self.K_consensus = 0.18

        root = Path(__file__).resolve().parents[1]
        mission_file = root / "artifacts" / "planning" / "mission_waypoints.json"
        with mission_file.open("r", encoding="utf-8") as f:
            mission = json.load(f)
        self.mission = mission["waypoints"]
        self.route_total_sec = float(mission.get("total_time_sec", self.mission[-1]["t_sec"] if self.mission else 0.0))

        # 不再固定10秒，按路线长度决定；仅保留安全超时兜底
        self.hard_timeout_sec = max(self.route_total_sec + 20.0, 40.0)
        self.finish_radius_m = 1.8

        self.states: Dict[int, UAVState] = {}
        self.takeoff_active = False
        self.takeoff_t0 = None
        self.mission_finished = False

        self.state_publishers = {uid: self.create_publisher(Odometry, f"/inspection/uav{uid}/state", 10) for uid in self.uav_ids}
        self.setpoint_publishers = {uid: self.create_publisher(TrajectorySetpoint, f"/px4_{uid}/fmu/in/trajectory_setpoint", 10) for uid in self.uav_ids}
        self.offboard_mode_publishers = {uid: self.create_publisher(OffboardControlMode, f"/px4_{uid}/fmu/in/offboard_control_mode", 10) for uid in self.uav_ids}
        self.cmd_publishers = {uid: self.create_publisher(VehicleCommand, f"/px4_{uid}/fmu/in/vehicle_command", 10) for uid in self.uav_ids}
        self.preview_publishers = {uid: self.create_publisher(PoseStamped, f"/inspection/uav{uid}/formation_preview", 10) for uid in self.uav_ids}

        self.fleet_state_pub = self.create_publisher(String, "/inspection/fleet_state", 10)
        self.collision_alert_pub = self.create_publisher(String, "/inspection/collision_alert", 10)

        for uid in self.uav_ids:
            self.create_subscription(VehicleOdometry, f"/px4_{uid}/fmu/out/vehicle_odometry", lambda msg, u=uid: self._odom_cb(u, msg), 20)

        self.create_subscription(Bool, "/inspection/takeoff_cmd", self._takeoff_cb, 10)

        self.create_timer(1.0, self._publish_state_1hz)
        self.create_timer(0.1, self._control_loop_10hz)

        self.get_logger().info(f"FormationCoordinator ready. route_total_sec={self.route_total_sec:.2f}")

    def _odom_cb(self, uid: int, msg: VehicleOdometry):
        self.states[uid] = UAVState(
            pos=np.array([msg.position[0], msg.position[1], -msg.position[2]], dtype=float),
            vel=np.array([msg.velocity[0], msg.velocity[1], -msg.velocity[2]], dtype=float),
            stamp_sec=self.get_clock().now().nanoseconds / 1e9,
        )

    def _takeoff_cb(self, msg: Bool):
        if msg.data and not self.takeoff_active:
            self.takeoff_active = True
            self.takeoff_t0 = self.get_clock().now().nanoseconds / 1e9
            self.mission_finished = False
            self.get_logger().info("Takeoff command received. mission start.")

    def _publish_state_1hz(self):
        now = self.get_clock().now().to_msg()
        for uid in self.uav_ids:
            if uid not in self.states:
                continue
            st = self.states[uid]
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.header.stamp = now
            odom.child_frame_id = f"uav{uid}"
            odom.pose.pose.position.x = float(st.pos[0])
            odom.pose.pose.position.y = float(st.pos[1])
            odom.pose.pose.position.z = float(st.pos[2])
            odom.twist.twist.linear.x = float(st.vel[0])
            odom.twist.twist.linear.y = float(st.vel[1])
            odom.twist.twist.linear.z = float(st.vel[2])
            self.state_publishers[uid].publish(odom)
        self._detect_collision_risk()

    def _detect_collision_risk(self):
        ids = [uid for uid in self.uav_ids if uid in self.states]
        alerts = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                ui, uj = ids[i], ids[j]
                dij = float(np.linalg.norm(self.states[ui].pos - self.states[uj].pos))
                if dij < 2.5:
                    alerts.append(f"risk:uav{ui}-uav{uj},d={dij:.2f}")
        if alerts:
            msg = String(); msg.data = " | ".join(alerts)
            self.collision_alert_pub.publish(msg)

    def _lookup_leader_ref(self, t_sec: float) -> np.ndarray:
        if len(self.mission) == 0:
            return np.array([0.0, 0.0, 20.0])
        ts = np.array([wp["t_sec"] for wp in self.mission], dtype=float)
        idx = int(np.argmin(np.abs(ts - t_sec)))
        wp = self.mission[idx]
        return np.array([wp["x"], wp["y"], wp["z"]], dtype=float)

    def _final_leader_ref(self) -> np.ndarray:
        if len(self.mission) == 0:
            return np.array([0.0, 0.0, 20.0])
        wp = self.mission[-1]
        return np.array([wp["x"], wp["y"], wp["z"]], dtype=float)

    def _consensus_correction(self, uid: int) -> np.ndarray:
        if uid not in self.states:
            return np.zeros(3)
        i = self.uav_ids.index(uid)
        corr = np.zeros(3)
        for j, uj in enumerate(self.uav_ids):
            if self.A[i, j] <= 0 or uj not in self.states:
                continue
            corr += self.A[i, j] * (self.states[uj].pos - self.states[uid].pos)
        return self.K_consensus * corr

    def _publish_offboard_mode(self, uid: int):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_publishers[uid].publish(msg)

    def _publish_setpoint(self, uid: int, pos: np.ndarray):
        sp = TrajectorySetpoint()
        sp.position = [float(pos[0]), float(pos[1]), float(-pos[2])]
        sp.yaw = float(math.atan2(pos[1], pos[0]))
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_publishers[uid].publish(sp)

        pv = PoseStamped()
        pv.header.frame_id = "map"
        pv.header.stamp = self.get_clock().now().to_msg()
        pv.pose.position.x = float(pos[0]); pv.pose.position.y = float(pos[1]); pv.pose.position.z = float(pos[2])
        pv.pose.orientation.w = 1.0
        self.preview_publishers[uid].publish(pv)

    def _publish_land_cmd(self, uid: int):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.target_system = uid
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_publishers[uid].publish(cmd)

    def _is_route_finished(self, elapsed: float) -> bool:
        if elapsed < self.route_total_sec:
            return False
        leader_goal = self._final_leader_ref()
        for uid in self.uav_ids:
            if uid not in self.states:
                return False
            if uid == self.leader_id:
                goal = leader_goal
            else:
                goal = leader_goal + self.follow_offsets[uid]
            d = float(np.linalg.norm(self.states[uid].pos - goal))
            if d > self.finish_radius_m:
                return False
        return True

    def _control_loop_10hz(self):
        if not self.takeoff_active:
            return

        elapsed = self.get_clock().now().nanoseconds / 1e9 - float(self.takeoff_t0)
        leader_ref = self._lookup_leader_ref(elapsed)

        for uid in self.uav_ids:
            self._publish_offboard_mode(uid)
            target = leader_ref + (self.follow_offsets[uid] if uid != self.leader_id else 0.0) + self._consensus_correction(uid)
            self._publish_setpoint(uid, target)

        finished = self._is_route_finished(elapsed)
        timeout = elapsed >= self.hard_timeout_sec

        fleet = String()
        fleet.data = json.dumps({
            "mode": "mission",
            "elapsed_sec": round(float(elapsed), 3),
            "route_total_sec": self.route_total_sec,
            "finished": finished,
            "hard_timeout": timeout,
            "leader_ref": leader_ref.tolist(),
        }, ensure_ascii=False)
        self.fleet_state_pub.publish(fleet)

        if (finished or timeout) and not self.mission_finished:
            for uid in self.uav_ids:
                self._publish_land_cmd(uid)
            self.takeoff_active = False
            self.mission_finished = True
            reason = "route_completed" if finished else "hard_timeout"
            self.get_logger().info(f"Mission end: {reason}. LAND command published.")


def main():
    rclpy.init()
    node = FormationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
