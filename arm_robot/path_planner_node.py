import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # Publishing to MotionControllerNode
        self.trajectory_publisher = self.create_publisher(
            RobotTrajectory, 'planned_trajectory', 1
        )

        # Joint list
        self.JOINT_NAMES = [
            'ur20_shoulder_pan_joint',
            'ur20_shoulder_lift_joint',
            'ur20_elbow_joint',
            'ur20_wrist_1_joint',
            'ur20_wrist_2_joint',
            'ur20_wrist_3_joint'
        ]

        # Publishes the test trajectory every 10 seconds
        self.timer = self.create_timer(10.0, self.plan_and_publish)

        self.get_logger().info("Path Planner Node has been started. It's waiting for poses in the /target_pose topic.")
    
    def plan_and_publish(self):
        # Creates a RobotTrajectory and publishes it
        joint_traj = JointTrajectory()
        joint_traj.header.stamp = self.get_clock().now().to_msg()
        joint_traj.joint_names = self.JOINT_NAMES

        # Rest pose
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * len(self.JOINT_NAMES)
        point1.time_from_start = Duration(sec=1, nanosec=0)

        # Test pose
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.5, -0.5, 0.5, 0.0, 0.0] 
        point2.time_from_start = Duration(sec=5, nanosec=0)

        # Back to rest pose
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0] * len(self.JOINT_NAMES)
        point3.time_from_start = Duration(sec=9, nanosec=0)

        joint_traj.points.append(point1)
        joint_traj.points.append(point2)
        joint_traj.points.append(point3)

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = joint_traj

        # Publishes the trajectory
        self.trajectory_publisher.publish(robot_traj)
        self.get_logger().info("--> Mock RobotTrajectory published.")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()