import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher = self.create_publisher(JointTrajectory, '/milltap_controller/joint_trajectory', 10)
        self.send_trajectory()

    def send_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, -0.5, 1.0, -1.0]  # Target positions for each joint
        point.velocities = [0.0] * 5
        point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()  # Time to reach positions
        
        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_mover = JointMover()
    rclpy.spin_once(joint_mover)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
