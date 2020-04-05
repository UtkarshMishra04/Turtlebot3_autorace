#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,LaserScan,Imu,CameraInfo,PointCloud,Range
from math import pow, atan2, sqrt, asin, pi
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)
        return yaw

class TurtleBot:

    def __init__(self):

        rospy.init_node('my_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('cmd_vel',Twist, queue_size=10)

        self.scan_data = rospy.Subscriber('scan',LaserScan,self.modifyScan)

        self.cam_data = rospy.Subscriber('camera/image',Image,self.modifyImage)
        self.bridge = CvBridge()

        self.pose_subscriber = rospy.Subscriber('odom',Odometry, self.update_config)

        self.config = Odometry()
        self.theta = 0
        self.rate = rospy.Rate(10)

        self.camera_data = None
        self.height = 0
        self.width = 0
        self.image = None

        self.minang = 0
        self.maxang = 0
        self.incrementang = 0
        self.incrementtime = 0
        self.rangemin = 0
        self.rangemax = 0
        self.ranges = []

    def modifyImage(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.camera_data = data
        self.height = data.height
        self.width = data.width
        self.image = cv_image

        cv2.imshow('Image1',self.image)
        cv2.waitKey(0)

        raw_input("Press Enter to continue...")




    def update_config(self, data):
        self.config = data
        self.theta = quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)


    def modifyScan(self, data):
        self.scan_data = data
        self.minang = data.angle_min
        self.maxang = data.angle_max
        self.incrementang = data.angle_increment
        self.incrementtime = data.time_increment
        self.rangemin = data.range_min
        self.rangemax = data.range_max
        self.ranges = data.ranges
        self.ranges = np.array(self.ranges)
        self.angles = np.arange(self.minang,self.maxang+0.1*self.incrementang,self.incrementang)

        for i in range (0,360):
            if i>180:
                self.angles[i] = self.angles[i] -2*pi


    def disp_all(self):

        vel_msg = Twist()
        i=0;

        while(i<100):

            vel_msg.linear.x = 2
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 2

        
            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

            print(self.config.twist)

            i=i+1

        rospy.spin()

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.pose.pose.position.x - self.config.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.config.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return 0.05 * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        if (self.ranges != []):
            obstacle = np.min(self.ranges)
            if np.argmin(self.ranges) <= 90 or np.argmin(self.ranges) >= 270:
                angleobs = self.angles[np.argmin(self.ranges)]
                if obstacle < (self.maxang + self.minang)/5:
                    angle = atan2(goal_pose.pose.pose.position.y - self.config.pose.pose.position.y, goal_pose.pose.pose.position.x - self.config.pose.pose.position.x)
                    choice1 = angleobs+pi/2
                    choice2 = angleobs-pi/2
                
                    if abs(abs(choice1)-abs(angle)) <= abs(abs(choice2)-abs(angle)):
                        angle = choice1
                    else:
                        angle = choice2

                    print("Obstacle avoiding!")

                else:
                    angle = atan2(goal_pose.pose.pose.position.y - self.config.pose.pose.position.y, goal_pose.pose.pose.position.x - self.config.pose.pose.position.x)
                    print("Goal tracking!")
            else:
                angle = atan2(goal_pose.pose.pose.position.y - self.config.pose.pose.position.y, goal_pose.pose.pose.position.x - self.config.pose.pose.position.x)
                print("Goal tracking!")
        else:
            angle = atan2(goal_pose.pose.pose.position.y - self.config.pose.pose.position.y, goal_pose.pose.pose.position.x - self.config.pose.pose.position.x)
            print("Goal tracking!")

        #angle = atan2(goal_pose.pose.pose.position.y - self.config.pose.pose.position.y, goal_pose.pose.pose.position.x - self.config.pose.pose.position.x)

        return angle

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return 1 * (self.steering_angle(goal_pose) - self.theta)

    def move_to_origin(self):
        goal_pose = Odometry()
        goal_pose.pose.pose.position.x = 2
        goal_pose.pose.pose.position.y = 2
        goal_pose.pose.pose.position.z = 0
        goal_pose.pose.pose.orientation.x = 0
        goal_pose.pose.pose.orientation.y = 0
        goal_pose.pose.pose.orientation.z = 0
        goal_pose.pose.pose.orientation.w = 0

        goal_pose.twist.twist.linear.x = 0
        goal_pose.twist.twist.linear.y = 0
        goal_pose.twist.twist.linear.z = 0
        goal_pose.twist.twist.angular.x = 0
        goal_pose.twist.twist.angular.y = 0
        goal_pose.twist.twist.angular.z = 0        


        distance_tolerance = 0.05

        vel_msg = Twist()

        print(self.euclidean_distance(goal_pose))

        var_cam=1

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            print("Hey! we are in loop!")
            print(self.config.pose.pose.position.x)
            print(self.config.pose.pose.position.y)
            print(self.steering_angle(goal_pose))
            print(self.theta)
            #print(self.image.shape)


            #if var_cam == 1:
            #    cv2.imshow('Image1',self.image)
            #    cv2.waitKey(0)
            #    cv2.destroyAllWindows()
            #    var_cam = 0
            #    raw_input("Press Enter to continue...")

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        print("We are done!")

        # If we press control + C, the node will stop.
        #rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move_to_origin()
    except rospy.ROSInterruptException:
        print("Mistake!")
        pass

    #cv2.destroyAllWindows()