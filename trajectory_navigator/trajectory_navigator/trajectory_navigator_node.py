import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy.node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
import rclpy.qos

class TrajectoryNavigatorNode(rclpy.node.Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self._goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self._amcl_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._poseCb, rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        '''
        self._goals_list = [
            [-1.94, -0.44, 0.0, 0.0, 0.0, 0.0, 1.0],
            [-0.53, -1.86, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.56, -0.07, 0.0, 0.0, 0.0, 0.0, 1.0]
        ]
        '''

        self._goals_list = [
            #[-14.212, -16.560, 0.0, 0.0, 0.0, 0.0, 1.0],
            [13.605, -25.650, 0.0, 0.0, 0.0, 0.0, 1.0],
            [4.419, -28.249, 0.0, 0.0, 0.0, 0.0, 1.0],
            [6.911, -34.374, 0.0, 0.0, 0.0, 0.0, 1.0],
            [-0.457, -25.683, 0.0, 0.0, 0.0, 0.0, 1.0],
            [-0.220, -16.687, 0.0, 0.0, 0.0, 0.0, 1.0]
        ]

        self._goal_index = 0
        self._navigating = False
        self._amcl_pose = None
        self._current_goal = None

        self._x_margin = 0.5
        self._y_margin = 0.5

        self._timer = self.create_timer(1, self._step)

    def _step(self):
        self._current_goal = self._get_goal(self._goals_list[self._goal_index])

        print('Moving to: ' + str(self._goals_list[self._goal_index]))
        self._send_goal()

        if (self._amcl_pose == None):
            return

        if (((self._current_goal.pose.position.x) > ((self._amcl_pose.position.x) - self._x_margin)) and
            (self._current_goal.pose.position.x < ((self._amcl_pose.position.x) + self._x_margin)) and
            ((self._current_goal.pose.position.y) > ((self._amcl_pose.position.y) - self._y_margin)) and
            ((self._current_goal.pose.position.y) < ((self._amcl_pose.position.y) + self._y_margin))):
            self._goal_index += 1
            if (self._goal_index == len(self._goals_list)):
                self._goal_index = 0

    def _poseCb(self, msg):
        amcl_pose = Pose()
        self._amcl_pose = msg.pose.pose

        print("Received amcl pose")

    def _send_goal(self):
        self._goal_pub.publish(self._current_goal)

    def _get_goal(self, pose):
        goal_pose = PoseStamped()
        
        
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]

        goal_pose.pose.orientation.x = pose[3]
        goal_pose.pose.orientation.y = pose[4]
        goal_pose.pose.orientation.z = pose[5]
        goal_pose.pose.orientation.w = pose[6]

        return goal_pose

def main(args=None):
    
    rclpy.init(args=args)

    trajectory_navigator_node = TrajectoryNavigatorNode('trajectory_navigator_node')

    rclpy.spin(trajectory_navigator_node)

if __name__ == "__main__":
    main()