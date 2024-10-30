#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Control the joints using the keyboard!
--------------------------------------
Keys:
    2/@ : Increase/Decrease joint2
    3/# : Increase/Decrease joint3
    4/$ : Increase/Decrease joint4
    5/% : Increase/Decrease joint5
    6/^ : Increase/Decrease joint6

CTRL-C to quit
"""

# Default joint positions
joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Default velocities
joint_accelerations = [0.0, 0.0, 0.0, 0.0, 0.0]  # Default accelerations

# Mapping of key inputs to joint index and movement
key_bindings = {
    '2': (0, 0.1),  # Increase joint2
    '@': (0, -0.1),  # Decrease joint2
    '3': (1, 0.1),  # Increase joint3
    '#': (1, -0.1),  # Decrease joint3
    '4': (2, 0.1),  # Increase joint4
    '$': (2, -0.1),  # Decrease joint4
    '5': (3, 0.1),  # Increase joint5
    '%': (3, -0.1),  # Decrease joint5
    '6': (4, 0.1),  # Increase joint6
    '^': (4, -0.1),  # Decrease joint6
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def check_collision(actual_positions):
    # Example collision detection logic based on actual positions
    # Define collision thresholds for each joint
    collision_thresholds = {
        'joint2': (-0.35, 0.35),
        'joint3': (-1.74533, 2.0944),
        'joint5': (-0.21, 0.21),
        'joint6': (-0.19, 0.19),
        'joint4': None  # No limits for joint4
    }

    # Check for joint limits
    for i, joint_name in enumerate(['joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
        if collision_thresholds[joint_name] is not None:
            lower, upper = collision_thresholds[joint_name]
            if not (lower <= actual_positions[i] <= upper):
                return True  # Collision detected

    return False  # No collision

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher = self.create_publisher(JointTrajectory, '/milltap_controller/joint_trajectory', 10)
        self.collision_publisher = self.create_publisher(Bool, '/collision_status', 10)
        self.joint_names = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Subscribe to joint states
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.actual_joint_positions = [0.0] * len(self.joint_names)  # Initialize actual joint positions

    def joint_state_callback(self, msg):
        # Update actual joint positions from the joint state message
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                index = self.joint_names.index(name)
                self.actual_joint_positions[index] = msg.position[i]

    def update_joint_positions(self, joint_positions):
        # Use actual joint positions for collision detection
        collision_detected = check_collision(self.actual_joint_positions)

        # Publish collision status
        collision_msg = Bool()
        collision_msg.data = collision_detected
        self.collision_publisher.publish(collision_msg)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = joint_velocities
        point.accelerations = joint_accelerations
        point.time_from_start.sec = 1

        traj_msg.points = [point]
        self.publisher.publish(traj_msg)
        print(f"Collision: {collision_detected}")
        print(f"Updated joint positions: {joint_positions}, velocities: {joint_velocities}, accelerations: {joint_accelerations}")
def main(args=None):
    rclpy.init(args=args)
    node = JointMover()
    
    try:
        print(msg)
        while True:
            key = getKey()
            if key in key_bindings.keys():
                joint_index, increment = key_bindings[key]
                joint_positions[joint_index] += increment
                node.update_joint_positions(joint_positions)
            elif key == '\x03':  # CTRL-C to exit
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
