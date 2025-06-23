#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DistanceMaintainer:
    def __init__(self):
        rospy.init_node("distance_regulator")

        # Параметры регулятора
        self.desired_range = 1.2
        self.gain_p = 0.8
        self.gain_i = 0.0
        self.gain_d = 0.2
        self.velocity_limit = 0.6
        self.view_angle_deg = 5.0

        # Внутреннее состояние
        self.accumulated_error = 0.0
        self.last_range = None
        self.previous_time = rospy.Time.now()

        # Подписка и публикация
        self.lidar_sub = rospy.Subscriber("/scan_filtered", LaserScan, self.process_scan)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Регулятор расстояния инициализирован")
        rospy.loginfo(f"Цель: {self.desired_range} м, Угол обзора: ±{self.view_angle_deg}°")
        rospy.loginfo(f"Параметры PID: P={self.gain_p}, I={self.gain_i}, D={self.gain_d}")
        rospy.loginfo(f"Макс. скорость: {self.velocity_limit} м/с")

    def extract_central_range(self, scan_msg):
        angle_halfwidth = math.radians(self.view_angle_deg) / 2
        valid_ranges = []

        for idx, reading in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + idx * scan_msg.angle_increment

            if -angle_halfwidth <= angle <= angle_halfwidth:
                if scan_msg.range_min <= reading <= scan_msg.range_max:
                    if not np.isnan(reading) and not np.isinf(reading):
                        valid_ranges.append(reading)

        return min(valid_ranges) if valid_ranges else None

    def process_scan(self, scan_data):
        current_range = self.extract_central_range(scan_data)
        if current_range is None:
            self.velocity_pub.publish(Twist())  # остановка
            return

        now = rospy.Time.now()
        delta_t = (now - self.previous_time).to_sec()
        if delta_t <= 0:
            return

        # Расчёт ошибки и PID составляющих
        deviation = current_range - self.desired_range

        prop = self.gain_p * deviation
        self.accumulated_error += deviation * delta_t
        integral = self.gain_i * self.accumulated_error

        if self.last_range is not None:
            rate = (current_range - self.last_range) / delta_t
        else:
            rate = 0.0
        derivative = self.gain_d * rate

        # Командный сигнал
        output_speed = prop + integral + derivative
        output_speed = max(min(output_speed, self.velocity_limit), -self.velocity_limit)

        # Публикация команды
        motion_cmd = Twist()
        motion_cmd.linear.x = output_speed
        self.velocity_pub.publish(motion_cmd)

        self.last_range = current_range
        self.previous_time = now

if __name__ == "__main__":
    try:
        DistanceMaintainer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
