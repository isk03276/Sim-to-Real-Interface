import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import copy

class SensingManager:
    def __init__(self):
        self.ob = None
        rospy.Subscriber('/rb1_base_head_camera/color/image_rect_color', Image, self.image_callback)
        self.bridge = CvBridge()
        self.img = None
        self.get_img_flag = False

    def image_callback(self, msg):
        if self.img is None:
            self.img = self.convert_imgmsg_to_img(msg)
            self.get_img_flag = True

    def convert_imgmsg_to_img(self, msg):
        """
        :param msg: sensor_msgs.msgs.Image message
        :return: converted (cv2) image
        """
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = cv2.resize(img, (84,84))
        return img

    def robot_state_callback(self, msg):
        pass

    def get_image(self):
        """
        :return: current ob or None
        """
        if self.img is not None:
            copied_img = copy.deepcopy(self.img)
            self.img = None
            self.get_img_flag = False
            return copied_img
        else:
            None
