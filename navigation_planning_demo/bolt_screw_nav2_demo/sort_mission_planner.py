
"""
Mission-level navigation planner.

This node has no YOLO dependency. Needs to be integrated with the classifier from YOLO 
(object_queue).
"""
import math
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener


def yaw_to_quaternion(yaw: float):
    """Convert a yaw angle (radians) to a quaternion (x, y, z, w).
    Used when building PoseStamped goal messages for Nav2.
    """
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SortMissionPlanner(Node):
    def __init__(self):
        super().__init__('sort_mission_planner')

        # All poses are [x, y, yaw] in the odom frame and loaded from stations.yaml.
        self.declare_parameter('frame_id', 'odom') # map 
        self.declare_parameter('source_station', [1.0, 0.0, 0.0])
        self.declare_parameter('inspection_pose', [1.2, 0.0, 0.0])
        self.declare_parameter('bins.heart', [2.0, 0.8, 1.57])
        self.declare_parameter('bins.triangle', [2.0, 0.25, 1.57])
        self.declare_parameter('bins.bolt', [2.0, -0.25, 1.57])
        self.declare_parameter('bins.l_shape', [2.0, -0.8, 1.57])
        self.declare_parameter('return_home', [0.0, 0.0, 0.0])
        self.declare_parameter('object_queue', ['heart', 'triangle', 'bolt', 'l_shape']) # Defines which shapes to sort and in what order.
        self.declare_parameter('stop_between_goals', 1.0)
        self.declare_parameter('navigation_timeout_sec', 90.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.stop_between_goals = float(self.get_parameter('stop_between_goals').value)
        self.timeout = float(self.get_parameter('navigation_timeout_sec').value)
        self.object_queue = list(self.get_parameter('object_queue').value)

        self.source_station = list(self.get_parameter('source_station').value)
        self.inspection_pose = list(self.get_parameter('inspection_pose').value)
        self.return_home = list(self.get_parameter('return_home').value)

        # Dictionary mapping each shape name to its bin pose [x, y, yaw]
        self.bins: Dict[str, List[float]] = {
            'heart': list(self.get_parameter('bins.heart').value),
            'triangle': list(self.get_parameter('bins.triangle').value),
            'bolt': list(self.get_parameter('bins.bolt').value),
            'l_shape': list(self.get_parameter('bins.l_shape').value),
        }

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose') # Action client for sending navigation goals to Nav2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def pose(self, xyz: List[float]) -> PoseStamped:
        """Build a PoseStamped message from a [x, y, yaw] list.
        
        The frame_id is set from the parameter (default: odom).
        The timestamp is set to now so Nav2 does not reject it as stale.
        """
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(float(xyz[2]))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def navigate(self, name: str, pose_xyz: List[float]) -> bool:
        """Send a NavigateToPose goal and block until it succeeds, fails or times out.
        """
        self.get_logger().info(f'Navigating to {name}: {pose_xyz}')

        # Send the goal and wait for the action server to accept or reject it.
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(pose_xyz)
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {name}')
            return False

        # Poll for the result, checking the timeout on every iteration.
        result_future = goal_handle.get_result_async()
        start = self.get_clock().now()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed > self.timeout:
                self.get_logger().error(f'Goal timed out, cancelling: {name}')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            # Pause to simulate pick or drop action at the station.
            self.get_logger().info(f'Reached {name}')
            time.sleep(self.stop_between_goals)
            return True

        self.get_logger().error(f'Navigation failed for {name}, status={result.status}, result={result.result}')
        return False

    def run_mission(self) -> bool:
        """Execute the full sorting mission. 
        """

        self.get_logger().info('Starting standalone bolts/screws navigation mission.')
        self.get_logger().info('Detection is mocked by object_queue; YOLO can be integrated later by replacing that input.')

        for shape in self.object_queue:
            # Validate shape before navigating to avoid mid-mission failures.
            if shape not in self.bins:
                self.get_logger().error(f'Unknown shape "{shape}". Valid shapes: {list(self.bins.keys())}')
                return False
            if not self.navigate('source_station_pick_area', self.source_station): # Go to the source station to pick up the object
                return False
            self.get_logger().info(f'Mock pick: picked object classified as {shape}')
            if not self.navigate(f'{shape}_bin_drop_area', self.bins[shape]): # Go to the correct bin and drop the object
                return False
            self.get_logger().info(f'Mock drop: delivered {shape} to matching bin')

        self.navigate('home', self.return_home)
        self.get_logger().info('Standalone navigation mission finished.')
        return True


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = SortMissionPlanner()
    try:
        ok = node.run_mission()
        if not ok:
            node.get_logger().error('Mission ended with failure.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()