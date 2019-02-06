#!/usr/bin/env python
import rospy
import rospkg
import os.path
from visualization_msgs.msg import Marker
from simulator.core import Simulator


class SimulatorNode:
    def __init__(self):
        rospy.init_node('simulator', anonymous=False)

        self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # By default, look in the package config directory.
        config_dir = os.path.join(rospkg.RosPack().get_path('simulator'), 'config')
        config_file = rospy.get_param('~config_file', 
                                      os.path.join(config_dir, 'default.yaml'))

        self.sim = Simulator(config_file)

    def run(self, freq=15.):
        rate = rospy.Rate(freq)
        dt = 1./freq
        t = 0.
        
        # TODO read this in from a message.
        inputs = {}

        while not rospy.is_shutdown():
            self.sim.step(inputs, t, dt)

            markers = self.sim.get_markers_ros()
            for m in markers: 
                self.vis_pub.publish(m)

            rate.sleep()
            t += dt


def main():
    node = SimulatorNode()

    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
