import roslib; roslib.load_manifest('kinova_demo')
import rospy
import math
import actionlib
import kinova_msgs.msg
from control.control_manager import ControlManager


class ArmControlManager(ControlManager):
    def __init__(self):
        self.initial_joint_degrees = [160, 270, 270, 270, 180, 0]
        self.position_control_address = '/j2n6s300_driver/joints_action/joint_angles'
        self.position_control_client = actionlib.SimpleActionClient(self.position_control_address,
                                              kinova_msgs.msg.ArmJointAnglesAction)
        self.position_control_client.wait_for_server()

        self.velocity_control_address = '/j2n6s300_driver/in/joint_velocity'
        self.velocity_control_publisher = rospy.Publisher(self.velocity_control_address,
                                                         kinova_msgs.msg.JointVelocity, queue_size=10)

    def position_control(self, ac, info):
        """
        :param ac: joint angle
        :param info: if ac is radian then True else false
        """
        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        if info:
            angle_set = self.convert_angle(ac, info)
        else:
            angle_set = ac

        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]

        self.position_control_client.send_goal(goal)
        if self.position_control_client.wait_for_result(rospy.Duration(20.0)):
            return self.position_control_client.get_result()
        else:
            print('the joint angle action timed-out')
            self.position_control_client.cancel_all_goals()
            return None

    def velocity_control(self, ac, info):
        """
        :param ac: joint velocity control action
        :param info: if action is radian then True else False
        :return: None
        """
        goal = kinova_msgs.msg.JointVelocity()

        if info:
            angle_set = self.convert_angle(ac, info)
        else:
            angle_set = ac

        goal.joint1 = angle_set[0]
        goal.joint2 = angle_set[1]
        goal.joint3 = angle_set[2]
        goal.joint4 = angle_set[3]
        goal.joint5 = angle_set[4]
        goal.joint6 = angle_set[5]

        self.velocity_control_publisher.publish(goal)

    def reset(self):
        """
        set initial joint
        :return: None
        """
        is_radian = False
        self.position_control(self.initial_joint_degrees, is_radian)