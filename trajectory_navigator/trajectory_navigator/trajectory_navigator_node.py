import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy.node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class TrajectoryNavigatorNode(rclpy.node.Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self._nav_client_act = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._goals_list = [
            [-1.94, -0.44, 0.0, 0.0, 0.0, 0.0, 1.0],
            [-0.53, -1.86, 0.0, 0.0, 0.0, 0.0, 1.0]
        ]

        self._pose_index = 0

        self._poses_list = self._get_poses()

    def step(self):
        if (not  self._nav_client_act.wait_for_server()):
            self.get_logger().error("Action Server Unavailable")
            return

        goals_msg = NavigateToPose.Goal()
        goals_msg.pose = self._poses_list[self._pose_index]
        self._send_goal_future = self._nav_client_act.send_goal_async(goals_msg,feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _get_poses(self):
        poses = []
        goal_pose = PoseStamped()
        
        for goal in self._goals_list:
            print(goal)
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'

            goal_pose.pose.position.x = goal[0]
            goal_pose.pose.position.y = goal[1]
            goal_pose.pose.position.z = goal[2]

            goal_pose.pose.orientation.x = goal[3]
            goal_pose.pose.orientation.y = goal[4]
            goal_pose.pose.orientation.z = goal[5]
            goal_pose.pose.orientation.w = goal[6]

            poses.append(goal_pose)

        return poses
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Navigating to: ' +
                               '(' + str(self._goals_list[self._pose_index][0]) + ',' +
                               str(self._goals_list[self._pose_index][1]) + ',' +
                               str(self._goals_list[self._pose_index][2]) + ')')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result().result

        self.get_logger().info('Goal Reached')
        
        self._pose_index += 1
        if (self._pose_index >= len(self._goals_list)):
            self._pose_index = 0
        self.step()
    
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Distance Remaining: {0}'.format(feedback.distance_remaining))

def main(args=None):
    
    rclpy.init(args=args)

    trajectory_navigator_node = TrajectoryNavigatorNode('trajectory_navigator_node')
    trajectory_navigator_node.step()

    rclpy.spin(trajectory_navigator_node)

if __name__ == "__main__":
    main()