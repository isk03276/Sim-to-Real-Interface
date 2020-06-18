from control.arm_control_manager import ArmControlManager
from control.finger_control_manager import FingerControlManager
from policy.policy_manager import PolicyManager
from sensing.sensing_manager import SensingManager
import rospy


class Controller():
    def __init__(self):
        self.node = rospy.init_node('sim_to_real')
        self.armControlManager = ArmControlManager()
        self.fingerControlManager = FingerControlManager()
        self.policyManager = PolicyManager()
        self.sensingManager = SensingManager()

    def run(self):
        pass

    def test(self):
        self.fingerControlManager.position_control([0.7, 0.7, 0.7], None)

if __name__ == '__main__':
    controller = Controller
    controller.test()
