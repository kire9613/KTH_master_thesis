#!/usr/bin/env python

import rospy
from svea.track import EditableTrack


def main():
    rospy.init_node('rviz_trackeditor')

    # start track handler
    track = EditableTrack('', publish_track=True)
    track.start()
    rospy.spin()


if __name__ == '__main__':
    main()
