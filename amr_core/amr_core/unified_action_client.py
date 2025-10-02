#!/usr/bin/env python3

# Run and send “goto Goal2” once:
# ros2 run amr_core unified_action_client --ros-args -p command_type:=goto_goal -p goal_name:=Goal2 -p exit_on_result:=true
#
# Subscribe to goal poses and send gotoPoint on messages:
# ros2 run amr_core unified_action_client --ros-args -p enable_goal_pose_sub:=true -p goal_pose_topic:=goal_pose
# 
# Subscribe to initial poses and send localizeAtPoint on messages:
# ros2 run amr_core unified_action_client --ros-args -p enable_initialpose_sub:=true -p initialpose_topic:=initialpose
# 
# Execute a macro or dock:
# ros2 run amr_core unified_action_client --ros-args -p command_type:=execute_macro -p macro_name:=Macro4
# ros2 run amr_core unified_action_client --ros-args -p command_type:=dock -p exit_on_result:=true


import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amr_msgs.action import Action
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

GOTO_POINT_COMMAND = "doTask gotoPoint "
LOCALIZE_TO_POINT_COMMAND = "localizetopoint "
DEFAULT_GOAL_POSE_TOPIC = "goal_pose"
DEFAULT_INITIAL_POSE_TOPIC = "initialpose"


class UnifiedAmrActionClient(Node):
    def __init__(self):
        super().__init__("amr_unified_action_client")
        # Action client
        self._ac = ActionClient(self, Action, "action_server")

        # Parameters
        self.declare_parameter("enable_goal_pose_sub", False)
        self.declare_parameter("enable_initialpose_sub", False)
        self.declare_parameter("goal_pose_topic", DEFAULT_GOAL_POSE_TOPIC)
        self.declare_parameter("initialpose_topic", DEFAULT_INITIAL_POSE_TOPIC)
        self.declare_parameter("command_type", "none")  # none|goto_goal|execute_macro|dock
        self.declare_parameter("goal_name", "Goal1")
        self.declare_parameter("macro_name", "Macro1")
        self.declare_parameter("exit_on_result", False)

        # Subscriptions (optional)
        if self.get_parameter("enable_goal_pose_sub").get_parameter_value().bool_value:
            topic = self.get_parameter("goal_pose_topic").get_parameter_value().string_value
            self.create_subscription(PoseStamped, topic, self._on_goal_pose, 10)
            self.get_logger().info(f"Subscribed to goal pose: {topic}")

        if self.get_parameter("enable_initialpose_sub").get_parameter_value().bool_value:
            topic = self.get_parameter("initialpose_topic").get_parameter_value().string_value
            self.create_subscription(PoseWithCovarianceStamped, topic, self._on_initial_pose, 10)
            self.get_logger().info(f"Subscribed to initial pose: {topic}")

        # Fire a one-shot command from params if requested
        cmd = self.get_parameter("command_type").get_parameter_value().string_value
        if cmd == "goto_goal":
            name = self.get_parameter("goal_name").get_parameter_value().string_value
            self.goto_goal(name)
        elif cmd == "execute_macro":
            name = self.get_parameter("macro_name").get_parameter_value().string_value
            self.execute_macro(name)
        elif cmd == "dock":
            self.dock()

    # ========== High-level commands ==========
    def goto_goal(self, name: str):
        goal = Action.Goal()
        goal.command = "goto " + name
        goal.identifier = [f"Arrived at {name}"]
        self._send_goal(goal)

    def execute_macro(self, name: str):
        goal = Action.Goal()
        goal.command = "executeMacro " + name
        goal.identifier = [f"Completed macro {name}"]
        self._send_goal(goal)

    def dock(self):
        goal = Action.Goal()
        goal.command = "dock"
        goal.identifier = ["Arrived at dock"]
        self._send_goal(goal)

    # Pose-based commands (from subscriptions)
    def _on_goal_pose(self, msg: PoseStamped):
        x = int(msg.pose.position.x * 1000.0)
        y = int(msg.pose.position.y * 1000.0)
        deg = self._yaw_deg(msg.pose.orientation.w, msg.pose.orientation.x,
                            msg.pose.orientation.y, msg.pose.orientation.z)
        if deg > 180:
            deg -= 360
        coords = f"{x} {y} {int(deg)}"
        goal = Action.Goal()
        goal.command = GOTO_POINT_COMMAND + coords
        goal.identifier = ["Going to point"]
        self._send_goal(goal)

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x = int(p.x * 1000.0)
        y = int(p.y * 1000.0)
        deg = self._yaw_deg(q.w, q.x, q.y, q.z)
        if deg > 180:
            deg -= 360
        coords = f"{x} {y} {int(deg)} 0 0"  # xySpread=0 angleSpread=0
        goal = Action.Goal()
        goal.command = LOCALIZE_TO_POINT_COMMAND + coords
        goal.identifier = ["Localizing at point"]
        self._send_goal(goal)

    # ========== Action helpers ==========
    def _send_goal(self, goal: Action.Goal):
        self._ac.wait_for_server()
        fut = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().info("Goal Rejected!")
            return
        self.get_logger().info(f"Goal: {gh.goal_request.goal.command}")
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(result.res_msg)
        if self.get_parameter("exit_on_result").get_parameter_value().bool_value:
            rclpy.shutdown()

    def _on_feedback(self, fb_msg):
        self.get_logger().info(fb_msg.feedback.feed_msg)

    # ========== Math ==========
    def _yaw_deg(self, rw, rx, ry, rz) -> float:
        t3 = +2.0 * (rw * rz + rx * ry)
        t4 = +1.0 - 2.0 * (ry * ry + rz * rz)
        yaw = round(math.atan2(t3, t4), 5)  # radians
        return yaw * 57.296  # degrees (approx 180/pi)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedAmrActionClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()