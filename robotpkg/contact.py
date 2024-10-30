import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState  # For monitoring joint states
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
collision_threshold = 5.0  # Adjust based on realistic torque limits

# Mapping of key inputs to joint index and movement
key_bindings = {
    '2': (0, 0.1),
    '@': (0, -0.1),
    '3': (1, 0.1),
    '#': (1, -0.1),
    '4': (2, 0.1),
    '$': (2, -0.1),
    '5': (3, 0.1),
    '%': (3, -0.1),
    '6': (4, 0.1),
    '^': (4, -0.1),
}

# Define joint constraints based on your YAML configuration
joint_constraints = {
    0: (-0.35, 0.35),
    1: (-1.74533, 2.0944),
    2: (-float('inf'), float('inf')),
    3: (-0.21, 0.21),
    4: (-0.19, 0.19)
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
        
        self.joint_publishers = {
            0: self.create_publisher(JointTrajectory, '/joints2_controllers/joint_trajectory', 10),
            1: self.create_publisher(JointTrajectory, '/joints3_controllers/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/joints4_controllers/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/joints5_controllers/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/joints6_controllers/joint_trajectory', 10)
        }
        
        self.joint_names = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
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
        for i, effort in enumerate(msg.effort):
            if abs(effort) > collision_threshold:
                print(f"Collision detected at joint {i + 2} with effort {effort}")
                self.collision_detected = True
                # Stop movement if collision is detected
                self.stop_movement()
                break
        else:
            self.collision_detected = False

    def stop_movement(self):
        global joint_velocities, joint_accelerations
        joint_velocities = [0.0] * len(joint_velocities)
        joint_accelerations = [0.0] * len(joint_accelerations)
        self.update_all_joints(joint_positions)
        print("All joints stopped due to collision.")

    def update_joint_positions(self, joint_index, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_names[joint_index]]
        
        point = JointTrajectoryPoint()
        point.positions = [joint_positions[joint_index]]
        point.velocities = [joint_velocities[joint_index]]
        point.accelerations = [joint_accelerations[joint_index]]
        point.time_from_start.sec = 1

        self.joint_publishers[joint_index].publish(traj_msg)
        print(f"Updated joint {joint_index + 2} position: {joint_positions[joint_index]}")

    def update_all_joints(self, joint_positions):
        for i in range(len(joint_positions)):
            self.update_joint_positions(i, joint_positions)

def main(args=None):
    rclpy.init(args=args)
    node = JointMover()
    
    try:
        print(msg)
        while True:
            key = getKey()
            if node.collision_detected:
                print("Movement stopped due to collision.")
                break
            if key in key_bindings.keys():
                joint_index, increment = key_bindings[key]
                new_position = joint_positions[joint_index] + increment
                
                min_limit, max_limit = joint_constraints[joint_index]
                if min_limit <= new_position <= max_limit:
                    joint_positions[joint_index] = new_position
                    joint_velocities[joint_index] = 0.1
                    joint_accelerations[joint_index] = 0.1
                    node.update_joint_positions(joint_index, joint_positions)
                else:
                    print(f"Joint {joint_index + 2} out of bounds: {new_position:.2f} (min: {min_limit}, max: {max_limit})")
            elif key == '\x03':
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
