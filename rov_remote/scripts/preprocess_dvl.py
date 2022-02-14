#!/usr/bin/python3

### Python
import numpy as np
import time
from math import sqrt
### ROS
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

from nortek_dvl.msg import CurrentProfile
### kalman filter from: https://github.com/milsto/robust-kalman 
import sys
sys.path.insert(0, '/home/lin/develop/dev/robust-kalman/robust_kalman/')
from robust_kalman import RobustKalman


class DVLFilter:
    def __init__(self):
        rospy.init_node('DVLFilter')
        self.node_name = rospy.get_name()
        self.get_params()

        self.init_subscribers()
        self.init_publishers()

        self.estimated_soundSpeed = None
        self.original_soundSpeed = None
        self.init_depth = None
        self.first_depth = True
        self.init_count = 5
        self.count = 0

        #### kalman filter for velocity
        F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32)                 # State transition matrix
        H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32)                 # Observation matrix
        P0 = np.ones((3, 3), np.float32) * 0.001                                    # Initial state covariance matrix
        Q0 = np.array([[0.0008, 0, 0], [0, 0.0008, 0], [0, 0, 0.0005]], np.float32) # (Initial) state noise covariance
        R0 = np.array([[self.observation_noise**2, 0, 0], 
                       [0, self.observation_noise**2, 0], 
                       [0, 0, self.observation_noise**2]], np.float32)              # (Initial) observation noise covariance

        # Create instance of the Kalman filters...
        x0_kalman = np.array([[0, 0, 0]], np.float32).T
        self.kalman_linear = RobustKalman(F, None, H, x0_kalman, P0, Q0, R0, use_robust_estimation=False, use_adaptive_statistics=False)

        ### kalman filter for depth 
        F_1 = np.array([[1]], np.float32)
        H_1 = np.array([[1]], np.float32)           # Observation matrix
        P0_1 = np.ones((1,1), np.float32) * 0.001   # Initial state covariance matrix
        Q0_1 = np.array([[0.01]], np.float32)       # (Initial) state noise covariance
        R0_1 = np.array([[0.25**2]], np.float32)     # (Initial) observation noise covariance

        # Create instance of the Kalman filters...
        x0_kalman_1 = np.array([[0]], np.float32).T
        self.kalman_linear_1 = RobustKalman(F_1, None, H_1, x0_kalman_1, P0_1, Q0_1, R0_1, use_robust_estimation=False, use_adaptive_statistics=False)

        while not rospy.is_shutdown():
            rospy.spin()

    def get_params(self):
      ### parameters
      self.use_estimaed_cov = rospy.get_param('~use_estimaed_cov', False)
      self.standard_soundSpeed = int(rospy.get_param('~standard_soundSpeed', "1500"))
      self.observation_noise = float(rospy.get_param('~observation_noise', '0.005'))
      self.air_pressure = float(rospy.get_param('~air_pressure', '0.00'))
      ### subscribed rostopics
      self.topic_dvl = rospy.get_param('~topic_dvl', '/twist')
      self.topic_currentProfile = rospy.get_param('~topic_filtered', '/current_profile')
      ### published rostopics
      self.topic_filtered = rospy.get_param('~topic_filtered', '/twist_filtered')
      self.topic_depth = rospy.get_param('~topic_depth', '/depth_filtered')

    def init_subscribers(self):
      rospy.Subscriber(self.topic_dvl, TwistWithCovarianceStamped, self.velocityCallback)
      rospy.Subscriber(self.topic_currentProfile, CurrentProfile, self.currentProfileCallback)

    def init_publishers(self):
      self.filtered_msg = TwistWithCovarianceStamped()
      self.filtered_pub = rospy.Publisher(self.topic_filtered, TwistWithCovarianceStamped, queue_size=10)

      self.depth_msg = PoseWithCovarianceStamped()
      self.depth_pub = rospy.Publisher(self.topic_depth, PoseWithCovarianceStamped, queue_size=10)

      self.depth_msg_1 = Float32()
      self.depth_pub_1 = rospy.Publisher("/test/normal_depth", Float32, queue_size=10)
      self.depth_msg_2 = Float32()
      self.depth_pub_2 = rospy.Publisher("/test/flitered_depth", Float32, queue_size=10)


    def soundSpeedCorrection(self, velocity):
        if self.estimated_soundSpeed is None:
            rospy.logwarn('%s - No sound speed correction.' % self.node_name)
            return velocity

        return velocity * self.estimated_soundSpeed / self.original_soundSpeed

    def velocityKalmanFilter(self, velocity_x, velocity_y, velocity_z):
      measurement = np.array([[velocity_x, velocity_y, velocity_z]], np.float32).T

      start_time = time.time()
      self.kalman_linear.time_update()
      self.kalman_linear.measurement_update(measurement)
      # print("---KF time: %s s ---" % (time.time() - start_time))

    def depthKalmanFilter(self, depth):
      measurement = np.array([[depth]], np.float32).T

      start_time = time.time()
      self.kalman_linear_1.time_update()
      self.kalman_linear_1.measurement_update(measurement)
      # print("---KF time: %s s ---" % (time.time() - start_time))

    def currentProfileCallback(self, msg):
      ### correct sound speed based on temperature
      self.original_soundSpeed = msg.sound_speed

      temperature = msg.temperature
      self.estimated_soundSpeed = self.standard_soundSpeed * sqrt( (273.15+temperature) / 273.15)

      ### convert depth into pose in odom frame
      if  self.first_depth:
        self.first_depth = False
        self.init_depth = msg.pressure - self.air_pressure
        
        # self.depth_msg_1.data = 0.0
        self.depthKalmanFilter(0.0)
      else:
        # self.depth_msg_1.data = self.init_depth - (msg.pressure - self.air_pressure)
        self.depthKalmanFilter(self.init_depth - (msg.pressure - self.air_pressure))

      self.depth_msg.header = msg.header
      self.depth_msg.header.frame_id = "odom"
      self.depth_msg.pose.pose.position.z = self.kalman_linear_1.current_estimate[0,0]
      self.depth_msg.pose.covariance[14] = self.kalman_linear_1.current_estimate_covariance[0,0]
      self.depth_pub.publish(self.depth_msg)

      #### test
      # self.depth_pub_1.publish(self.depth_msg_1)
      # self.depth_msg_2.data = self.kalman_linear_1.current_estimate[0,0]
      # self.depth_pub_2.publish(self.depth_msg_2)


    def velocityCallback(self, msg):
      ### correct sound speed
      v_x = self.soundSpeedCorrection(msg.twist.twist.linear.x)
      v_y = self.soundSpeedCorrection(msg.twist.twist.linear.y)
      v_z = self.soundSpeedCorrection(msg.twist.twist.linear.y)

      ### filter outliers and noise
      if abs(v_x)>5 or abs(v_y)>5 or abs(v_z)>5 :
        self.velocityKalmanFilter(self.filtered_msg.twist.twist.linear.x, self.filtered_msg.twist.twist.linear.y, self.filtered_msg.twist.twist.linear.z)
      else:
        self.velocityKalmanFilter(v_x, v_y, v_z)

      ### publish msg
      self.filtered_msg.header = msg.header
      self.filtered_msg.twist.twist.linear.x = self.kalman_linear.current_estimate[0,0]
      self.filtered_msg.twist.twist.linear.y = self.kalman_linear.current_estimate[1,0]
      self.filtered_msg.twist.twist.linear.z = self.kalman_linear.current_estimate[2,0]
      if self.use_estimaed_cov:
        self.filtered_msg.twist.covariance[0] = self.kalman_linear.current_estimate_covariance[0,0]
        self.filtered_msg.twist.covariance[7] = self.kalman_linear.current_estimate_covariance[1,1]
        self.filtered_msg.twist.covariance[14] = self.kalman_linear.current_estimate_covariance[2,2]      
      else:
        self.filtered_msg.twist.covariance[0] = msg.twist.covariance[0]
        self.filtered_msg.twist.covariance[7] = msg.twist.covariance[7]
        self.filtered_msg.twist.covariance[14] = msg.twist.covariance[14]

      self.filtered_pub.publish(self.filtered_msg)


if __name__ == '__main__':

  try:
      DVLFilterObject = DVLFilter()
  except rospy.ROSInterruptException:
      pass
