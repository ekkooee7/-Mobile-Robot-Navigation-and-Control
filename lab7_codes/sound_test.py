#!/usr/bin/env python
import roslib
import rospy, os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


# directory with sound assets - change as needed
soundAssets = '/home/shiloh/devel/audio_assets/'
# duration of yak throttle
throttle = 3 # seconds


if __name__ == '__main__':
    rospy.init_node('play', anonymous=True)
    soundhandle = SoundClient()
    rospy.sleep(1)
    num = int(1)
    volume = 1.0
    rospy.loginfo('Playing sound %i.' % num)
    soundhandle.play(num, volume)
    rospy.sleep(1)
