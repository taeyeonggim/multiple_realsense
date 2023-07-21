# -*- coding: utf-8 -*- 
"""ROS 기반의 logger 클래스를 포함한 파일입니다.

Author: tykim@plaif.com
Date: 2023-05-30
"""
import rospy


class ROSLogger():
    """ROS 기반의 logger 클래스입니다.

    Methods:
        debug(log_msg: str): 디버그 레벨의 로그를 출력
        info(log_msg: str): 정보 레벨의 로그를 출력
        warn(log_msg: str): 경고 레벨의 로그를 출력
        err(log_msg: str): 오류 레벨의 로그를 출력
        fatal(log_msg: str): 치명적 오류 레벨의 로그를 출력
    """

    def debug(self, log_msg):
        """디버그 레벨의 로그를 출력합니다.

        Args:
            log_msg (str): 로그 메세지

        Returns:
            None
        """
        if not isinstance(log_msg, str):
            raise TypeError("log_msg should be a string.")

        rospy.logdebug(log_msg)

    def info(self, log_msg):
        """정보 레벨의 로그를 출력합니다.

        Args:
            log_msg (str): 로그 메세지

        Returns:
            None
        """
        if not isinstance(log_msg, str):
            raise TypeError("log_msg should be a string.")

        rospy.loginfo(log_msg)

    def warn(self, log_msg):
        """경고 레벨의 로그를 출력합니다.

        Args:
            log_msg (str): 로그 메세지

        Returns:
            None
        """
        if not isinstance(log_msg, str):
            raise TypeError("log_msg should be a string.")

        rospy.logwarn(log_msg)

    def err(self, log_msg):
        """오류 레벨의 로그를 출력합니다.

        Args:
            log_msg (str): 로그 메세지

        Returns:
            None
        """
        if not isinstance(log_msg, str):
            raise TypeError("log_msg should be a string.")

        rospy.logerr(log_msg)

    def fatal(self, log_msg):
        """정보 레벨의 로그를 출력합니다.

        Args:
            log_msg (str): 로그 메세지

        Returns:
            None
        """
        if not isinstance(log_msg, str):
            raise TypeError("log_msg should be a string.")

        rospy.logfatal(log_msg)
        