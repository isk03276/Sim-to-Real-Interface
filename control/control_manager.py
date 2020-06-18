import math


class ControlManager:
    def position_control(self, ac, info):
        """
        :param ac: joint position control action
        :param info: if action is radian then True else False
        :return: None
        """
        raise NotImplementedError

    def velocity_control(self, ac, info):
        """
        :param ac: joint velocity control action
        :param info: if action is radian then True else False
        :return: None
        """
        raise NotImplementedError

    def convert_angle(self, ac, info):
        """
        if radian then degree else (degree) then radian
        :param ac: joint list
        :param info: if action is radian then True else False
        :return: converted_ac
        """
        converted_ac = None
        if info:  # radian
            converted_ac = list(map(math.degrees, ac))
        else:
            converted_ac = list(map(math.radians, ac))
        return converted_ac

    def reset(self):
        """
        set initial joint
        :return: None
        """
        raise NotImplementedError
