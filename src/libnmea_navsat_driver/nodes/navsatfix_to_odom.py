#! /usr/bin/env python3
import rospy
import utm
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class NavSatFixToOdom:
    def __init__(self):
        rospy.init_node('navsatfix_to_odom', anonymous=True)
        
        self.odom_pub = rospy.Publisher('/gps_odom', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)
        
        self.path = Path()
        self.path.header.frame_id = "gps"
        
        self.ref_easting = None
        self.ref_northing = None
        self.ref_altitude = None
        self.first_fix_received = False
        
        rospy.Subscriber('/fix', NavSatFix, self.navsatfix_callback)
        
    def navsatfix_callback(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
        # if msg.status.status != NavSatStatus.STATUS_GBAS_FIX:  # RTK Fix
            print("No fix")
            return
        
        if not self.first_fix_received:
            if msg.status.status != NavSatStatus.STATUS_NO_FIX:
            # if msg.status.status == NavSatStatus.STATUS_GBAS_FIX:  # RTK Fix
                utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
                self.ref_easting = utm_coords[0]
                self.ref_northing = utm_coords[1]
                self.ref_altitude = msg.altitude
                self.first_fix_received = True
                print("First fix received")
            else:
                return

        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
        easting, northing = utm_coords[0], utm_coords[1]
        
        enu_x = easting - self.ref_easting
        enu_y = northing - self.ref_northing
        enu_z = msg.altitude - self.ref_altitude
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "gps"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = enu_x
        odom.pose.pose.position.y = enu_y
        odom.pose.pose.position.z = enu_z
        odom.pose.covariance[0:3] = msg.position_covariance[0:3]
        odom.pose.covariance[6:9] = msg.position_covariance[3:6]
        odom.pose.covariance[12:15] = msg.position_covariance[6:9]
        
        self.odom_pub.publish(odom)
        
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        
        self.path.poses.append(pose)
        self.path.header.stamp = odom.header.stamp
        self.path_pub.publish(self.path)
        
if __name__ == '__main__':
    try:
        NavSatFixToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
