import rospy, time
import numpy as np
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from math import pi

def getTimeSafe():
    while True:
        # rospy may returns zero, so we loop until get a non-zero value.
        time = rospy.Time.now()
        if time != rospy.Time(0):
            return time

class Localization(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,
                queue_size = 1)
        self.rate = rospy.Rate(10)

        rospy.wait_for_service('global_localization')
        self.global_localization = rospy.ServiceProxy('global_localization',
                Empty)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.central_range = 0;
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy(
                'move_base/clear_costmaps', Empty)

    def execute(self, userdata):
        # reset AMCL localizer
        # self.clear_costmaps()
        self.global_localization()
        time.sleep(0.5) # wait for system to process

        # self turning
        duration = 12
        speed = 0.8
        tw = Twist()
        timelimit = getTimeSafe() + rospy.Duration(duration)
        while getTimeSafe() < timelimit:
            tw.angular.z = speed
            # if not np.isnan(self.central_range) and self.central_range > 3:
                # tw.linear.x = 0.25
            self.twist_pub.publish(tw)

        self.clear_costmaps()
        print "Done Localizing."
        return 'success'

    def scan_callback(self, msg):
        scan_data = msg.ranges
        self.central_range = scan_data[320]
