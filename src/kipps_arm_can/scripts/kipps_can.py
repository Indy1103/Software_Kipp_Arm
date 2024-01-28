#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import can
import struct

# Topic names
joint_traj_topic = "/move_group/fake_controller_joint_states"
joint_states_topics = "/joint_states_arm"

class Arm2Can:
    def __init__(self):
        rospy.init_node('arm2can', anonymous=True)

        self.node_id = 1
        self.joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_traj_sub = rospy.Subscriber(joint_traj_topic, JointState, self.joint_traj_msg_callback)
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        rospy.Timer(rospy.Duration(0.1), self.arm2can)

    def joint_traj_msg_callback(self, msg):
        joint_names = ["base_joint", "eef_joint", "link1_joint", "link2_joint", "link3_joint", "link4_joint"]
        for i, name in enumerate(joint_names):
            try:
                index = msg.name.index(name)
                self.joint_pos[i] = msg.position[index]
            except ValueError:
                print("error")  # Joint name not found

    def arm2can(self, event):
        can_messages = self.pos_2_float32(self.joint_pos)
        for msg in can_messages:
            self.bus.send(msg)

    def pos_2_float32(self, joint_pos):
        priority = 13
        frame_id = 0x11
        can_messages = []

        for i, pos in enumerate(joint_pos):
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            position_bytes = struct.pack('Bf', i+30, pos)
            can_msg = can.Message(arbitration_id=arbitration_id, data=position_bytes, is_extended_id=True)
            can_messages.append(can_msg)

        return can_messages

if __name__ == '__main__':
    try:
        arm_can_node = Arm2Can()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
