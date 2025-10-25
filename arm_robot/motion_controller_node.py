import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import RobotTrajectory
import time

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')

        # Subscribe to the topic which receives the trajectory from path planner node
        self.subscription = self.create_subscription(
            RobotTrajectory, 'planned_trajectory', self.execute_trajectory, 10
        )

        # Publishes to MoveIt's trajectory controller
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        self.get_logger().info("--> Motion controller node started")

    def execute_trajectory(self, traj_msg: RobotTrajectory):
        if not traj_msg.joint_trajectory.points:
            self.get_logger().warn("--> Empty trajectory received")
            return
        
        self.get_logger().info("--> Executing trajectory")

        self.joint_trajectory_pub.publish(traj_msg.joint_trajectory)

        self.get_logger().info("--> Trajectory published")

def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()