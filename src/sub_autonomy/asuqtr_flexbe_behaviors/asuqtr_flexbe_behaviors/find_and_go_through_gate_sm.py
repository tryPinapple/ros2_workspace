#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lqr_control_behaviors.go_to_object_at_camera_size_sm import Go_to_object_at_camera_sizeSM
from lqr_control_behaviors.turn_and_find_object_sm import Turn_and_find_objectSM
from lqr_control_states.time_x_forward_backward import TimeXForwardBackward
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jan 17 2021
@author: Thomas Germain
'''
class Find_and_go_through_gateSM(Behavior):
	'''
	Find and object, advance toward it while keeping the camera centred on it and then go through by advancing blindly for a while.
	'''


	def __init__(self):
		super(Find_and_go_through_gateSM, self).__init__()
		self.name = 'Find_and_go_through_gate'

		# parameters of this behavior
		self.add_parameter('object', 'person')
		self.add_parameter('object_score', 1)
		self.add_parameter('max_looking_time', 10)
		self.add_parameter('motor_throttle', 50)
		self.add_parameter('move_time', 5)

		# references to used behaviors
		self.add_behavior(Go_to_object_at_camera_sizeSM, 'Go_to_object_at_camera_size')
		self.add_behavior(Turn_and_find_objectSM, 'Turn_and_find_object')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 1023 58 
		# Advance forward for a while to go through an object

		# O 666 44 
		# Go to the object while centring it to the middle of the camera, if the object is lost go back to turn_and_find

		# ! 1346 92 
		# The AUV found the object, went to it and went forward for a while to go through it

		# O 249 56 
		# The AUV try to find an object and turn is yaw if it doesn't see it

		# ! 7 252 
		# The AUV was not able to change the angle, reached the maximum number of iterations inside a behavior or lost the object. 



	def create(self):
		# x:1306 y:100, x:169 y:222
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:250 y:90
			OperatableStateMachine.add('Turn_and_find_object',
										self.use_behavior(Turn_and_find_objectSM, 'Turn_and_find_object',
											parameters={'object': self.object, 'object_score': self.object_score, 'max_looking_time': self.max_looking_time}),
										transitions={'finished': 'Go_to_object_at_camera_size', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1025 y:94
			OperatableStateMachine.add('go_forward',
										TimeXForwardBackward(throttle=self.motor_throttle, moving_time=self.move_time),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Off})

			# x:666 y:89
			OperatableStateMachine.add('Go_to_object_at_camera_size',
										self.use_behavior(Go_to_object_at_camera_sizeSM, 'Go_to_object_at_camera_size',
											parameters={'object': self.object, 'object_score': self.object_score, 'object_percent_size': 80, 'max_looking_time': self.max_looking_time}),
										transitions={'finished': 'go_forward', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
