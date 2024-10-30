#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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

# Default joint positions, velocities, and accelerations
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

# Define joint constraints based on your YAML configuration
joint_constraints = {
    0: (-0.35, 0.35),  # joint2
    1: (-1.74533, 2.0944),  # joint3
    2: (-float('inf'), float('inf')),  # joint4 - No constraints in YAML
    3: (-0.21, 0.21),  # joint5
    4: (-0.19, 0.19)   # joint6
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher = self.create_publisher(JointTrajectory, '/milltap_controller/joint_trajectory', 10)
        self.joint_names = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def update_joint_positions(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = joint_velocities  # Set velocities
        point.accelerations = joint_accelerations  # Set accelerations
        point.time_from_start.sec = 1  # 1 second to reach target positions

        traj_msg.points = [point]
        self.publisher.publish(traj_msg)
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
                new_position = joint_positions[joint_index] + increment
                
                # Check against constraints
                min_limit, max_limit = joint_constraints[joint_index]
                if min_limit <= new_position <= max_limit:
                    joint_positions[joint_index] = new_position
                    # Update velocities and accelerations if needed
                    joint_velocities[joint_index] = 0.1  # Example: constant velocity
                    joint_accelerations[joint_index] = 0.1  # Example: constant acceleration
                    node.update_joint_positions(joint_positions)
                else:
                    print(f"Joint {joint_index + 2} out of bounds: {new_position:.2f} (min: {min_limit}, max: {max_limit})")
            elif key == '\x03':  # CTRL-C to exit
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
