#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty
from sensor_msgs.msg import JointState  # For monitoring joint states


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
collision_threshold = 5.0  # Adjust based on realistic torque limits


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

        # joint_publishers for each joint
        self.joint_publishers = {
            'joint2': self.create_publisher(JointTrajectory, '/joints2_controllers/joint_trajectory', 10),
            'joint3': self.create_publisher(JointTrajectory, '/joints3_controllers/joint_trajectory', 10),
            'joint4': self.create_publisher(JointTrajectory, '/joints4_controllers/joint_trajectory', 10),
            'joint5': self.create_publisher(JointTrajectory, '/joints5_controllers/joint_trajectory', 10),
            'joint6': self.create_publisher(JointTrajectory, '/joints6_controllers/joint_trajectory', 10)
        }

        # Subscribe to the joint states topic to monitor for collisions
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Replace with '/dynamic_joint_states' if needed
            self.joint_state_callback,
            10
        )
        self.collision_detected = False
    
    def joint_state_callback(self, msg):
        # Check for collision by monitoring the efforts of each joint
        print(f"joint ")
        for i, effort in enumerate(msg.effort):
            print(f"joint {i + 2} with effort {effort}")
            if abs(effort) > collision_threshold:
                print(f"Collision detected at joint {i + 2} with effort {effort}")
                self.collision_detected = True
                # Stop movement if collision is detected
                # self.stop_movement()
                # break
        else:
            self.collision_detected = False

    def update_joint_position(self, joint_name, position, velocity, acceleration):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [joint_name]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [velocity]
        point.accelerations = [acceleration]
        point.time_from_start.sec = 1  # 1 second to reach target position

        traj_msg.points = [point]
        self.joint_publishers[joint_name].publish(traj_msg)
        print(f"Updated {joint_name} to position: {position}, velocity: {velocity}, acceleration: {acceleration}")

def main(args=None):
    rclpy.init(args=args)
    node = JointMover()
    
    try:
        print(msg)
        while True:
            key = getKey()
            if key in key_bindings.keys():
                joint_index, increment = key_bindings[key]
                joint_name = f'joint{joint_index + 2}'
                new_position = joint_positions[joint_index] + increment
                
                # Check against constraints
                min_limit, max_limit = joint_constraints[joint_index]
                if min_limit <= new_position <= max_limit:
                    joint_positions[joint_index] = new_position
                    # Update velocities and accelerations if needed
                    joint_velocities[joint_index] = 0.1  # Example: constant velocity
                    joint_accelerations[joint_index] = 0.1  # Example: constant acceleration
                    node.update_joint_position(joint_name, new_position, joint_velocities[joint_index], joint_accelerations[joint_index])
                else:
                    print(f"{joint_name} out of bounds: {new_position:.2f} (min: {min_limit}, max: {max_limit})")
            elif key == '\x03':  # CTRL-C to exit
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
