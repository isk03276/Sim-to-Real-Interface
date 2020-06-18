from control.control_manager import ControlManager
import roslib; roslib.load_manifest('kinova_demo')
import rospy
import sys
import actionlib
import kinova_msgs.msg


class FingerControlManager(ControlManager):
    def __init__(self):
        self.initial_joint_degrees = [0, 0, 0]
        self.position_control_address = '/j2n6s300_driver/fingers_action/finger_positions'

        self.position_control_client = actionlib.SimpleActionClient(self.position_control_address,
                                              kinova_msgs.msg.SetFingersPositionAction)
        self.position_control_client.wait_for_server()

        self.finger_maxTurn = 6800  # max thread rotation for one finger

    def position_control(self, ac, info):
        """
        :param ac: joint position amount(0~1)
        :param info: None
        :return: None
        """
        goal = kinova_msgs.msg.SetFingersPositionGoal()

        converted_ac = self.convert_angle(ac, info)

        goal.fingers.finger1 = converted_ac[0]
        goal.fingers.finger2 = converted_ac[1]
        goal.fingers.finger3 = converted_ac[2]

        self.position_control_client.send_goal(goal)
        if self.position_control_client.wait_for_result(rospy.Duration(5.0)):
            return self.position_control_client.get_result()
        else:
            self.position_control_client.cancel_all_goals()
            rospy.logwarn('the gripper action timed-out')
            return None

    def velocity_control(self, ac, info):
        pass

    def convert_angle(self, ac, info):
        """
        convert ac to finger control action
        :param ac: joint position amount(0~1)
        :param info:  None
        :return: finger control action
        """
        assert max(ac) <= 1
        converted_ac = [x*self.finger_maxTurn for x in ac]

        converted_ac = [max(0.0, n) for n in converted_ac]
        converted_ac = [min(n, self.finger_maxTurn) for n in converted_ac]
        converted_ac = [float(n) for n in converted_ac]

        return converted_ac

    def reset(self):
        """
        set initial joint
        :return: None
        """
        self.position_control(self.initial_joint_degrees, None)

