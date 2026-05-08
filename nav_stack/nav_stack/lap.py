#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class SendPathWithControl(Node):
    def __init__(self):
        super().__init__("send_path_with_control")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")

        self.frame_id = self.get_parameter("frame_id").value
        self.action_name = self.get_parameter("action_name").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.progress_checker_id = self.get_parameter("progress_checker_id").value

        self.follow_client = ActionClient(self, FollowPath, self.action_name)
        self.goal_handle = None

        self.controller_change_state = self.create_client(
            ChangeState, "/controller_server/change_state"
        )
        self.costmap_change_state = self.create_client(
            ChangeState, "/local_costmap/local_costmap/change_state"
        )

    def wait_lifecycle_services(self):
        if not self.controller_change_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("controller_server change_state service not available")
            return False
        if not self.costmap_change_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("local_costmap change_state service not available")
            return False
        return True

    def change_state(self, client, transition_id, name):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error(f"Failed transition on {name}")
            return False

        if not future.result().success:
            self.get_logger().error(f"Transition refused on {name}")
            return False

        self.get_logger().info(f"{name}: transition {transition_id} success")
        return True

    def start_controller(self):
        if not self.wait_lifecycle_services():
            return False

        ok1 = self.change_state(
            self.costmap_change_state,
            Transition.TRANSITION_CONFIGURE,
            "local_costmap"
        )
        ok2 = self.change_state(
            self.controller_change_state,
            Transition.TRANSITION_CONFIGURE,
            "controller_server"
        )
        ok3 = self.change_state(
            self.costmap_change_state,
            Transition.TRANSITION_ACTIVATE,
            "local_costmap"
        )
        ok4 = self.change_state(
            self.controller_change_state,
            Transition.TRANSITION_ACTIVATE,
            "controller_server"
        )

        return ok1 and ok2 and ok3 and ok4

    def stop_controller(self):
        if not self.wait_lifecycle_services():
            return False

        self.cancel_goal()

        ok1 = self.change_state(
            self.controller_change_state,
            Transition.TRANSITION_DEACTIVATE,
            "controller_server"
        )
        ok2 = self.change_state(
            self.costmap_change_state,
            Transition.TRANSITION_DEACTIVATE,
            "local_costmap"
        )

        return ok1 and ok2

    def make_path(self, pts):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in pts:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        return path

    def send_path(self, pts):
        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowPath server not available")
            return

        goal = FollowPath.Goal()
        goal.path = self.make_path(pts)
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id

        future = self.follow_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f"distance_to_goal={fb.distance_to_goal:.2f}, speed={fb.speed:.2f}"
        )

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().error("Path goal rejected")
            return

        self.get_logger().info("Path goal accepted")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        self.get_logger().info(
            f"Finished. status={result.status}, error_code={result.result.error_code}"
        )

    def cancel_goal(self):
        if self.goal_handle is None:
            return
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info("Cancel sent")


def main():
    rclpy.init()
    node = SendPathWithControl()

    path_pts = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10, -10.0),
        (-6.0, -1.0),
        (0.0, 0.0),
        (10.0, 0.0),
        (10, -10.0),
        (-6.0, -1.0),
        (0.0, 0.0),
        (10.0, 0.0),
        (10, -10.0),
        (-6.0, -1.0),
    ]

    if not node.start_controller():
        node.get_logger().error("Could not start controller")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.send_path(path_pts)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_controller()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()