#!/usr/bin/env python
import rospy
import time
import smach_ros
from smach import State, StateMachine
from smach_ros import SimpleActionState

from NavAlert import NavSound
from Navigation import Navigation
from Docking import Docking
from ToDocking import ToDocking
from Undock import Undock
from Localization import Localization
from TakePicture import TakePicture
# from CancelGoal import CancelGoal

if __name__ == "__main__":
	rospy.init_node('state_machine_controller')

	sm = StateMachine(outcomes=['success','ready_to_dock','docking_complete','docking_failed', 'cancel_outcome'])
	# sm = StateMachine(outcomes=['done'])

	with sm:

		StateMachine.add('UNDOCKING', Undock(),
							transitions={'success': 'LOCALIZATION'})
		StateMachine.add('LOCALIZATION', Localization(),
							transitions={'success': 'NAVIGATION'})
		StateMachine.add('NAVIGATION', Navigation(),
							transitions={'done':'PLAY_SOUND'})
		StateMachine.add('TO_DOCKING', ToDocking(),
							transitions={'ready_to_dock': 'DOCKING'})
		StateMachine.add('DOCKING', Docking(),
							transitions={'docking_complete': None})
		StateMachine.add('TAKE_PICTURE', TakePicture(),
							transitions={'done': 'PLAY_SOUND'})
		StateMachine.add('PLAY_SOUND', NavSound(),
							transitions={'done': None})
		
		# StateMachine.add('CANCEL_TASK', CancelGoal(),
		# 					transitions={'cancel_outcome': 'TO_DOCKING'})

		time.sleep(1)
		sis = smach_ros.IntrospectionServer('server_name', sm, '/PHOTOBOT')
		sis.start()
		sm.execute()
