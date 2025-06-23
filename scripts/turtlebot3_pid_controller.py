#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller_node')
        
        self.target_distance = 1.0  
        self.kp = 0.8            
        self.ki = 0.0              
        self.kd = 0.2              
        self.max_speed = 0.5       
        self.sector_degrees = 5.0
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()
        self.prev_distance = None
        
        self.scan_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("PID Controller node started")
        rospy.loginfo(f"Target distance: {self.target_distance} m")
        rospy.loginfo(f"Front sector: {self.sector_degrees}°")
        rospy.loginfo(f"PID parameters: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        rospy.loginfo(f"Max speed: {self.max_speed} m/s")
    
    def get_front_distance(self, scan):
        sector_rad = math.radians(self.sector_degrees) / 2.0
        
        num_readings = len(scan.ranges)
        
        sector_distances = []
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            
            if angle > math.pi:
                angle -= 2 * math.pi
            elif angle < -math.pi:
                angle += 2 * math.pi
            
            if -sector_rad <= angle <= sector_rad:
                distance = scan.ranges[i]
                
                if not (np.isnan(distance) or np.isinf(distance)) and scan.range_min <= distance <= scan.range_max:
                    sector_distances.append(distance)
        
        if not sector_distances:
            return None

        return min(sector_distances)
    
    def scan_callback(self, scan):
        distance = self.get_front_distance(scan)
        
        if distance is None:
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return

        error = distance - self.target_distance

        P = self.kp * error

        self.integral += error * dt
        I = self.ki * self.integral

        if self.prev_distance is not None:
            derivative = (distance - self.prev_distance) / dt
        else:
            derivative = 0.0
        D = self.kd * derivative

        control_signal = P + I + D

        if control_signal > self.max_speed:
            control_signal = self.max_speed
        elif control_signal < -self.max_speed:
            control_signal = -self.max_speed

        cmd_vel = Twist()
        cmd_vel.linear.x = control_signal
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.prev_distance = distance
        self.last_time = current_time


if __name__ == '__main__':
    try:
        controller = PIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass