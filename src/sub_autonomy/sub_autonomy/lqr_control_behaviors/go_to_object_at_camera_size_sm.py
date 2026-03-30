#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from camera_states.object_screen_percent import ObjectScreenPercent
from lqr_control_behaviors.center_object_in_camera_middle_sm import Center_object_in_camera_middleSM
from lqr_control_states.time_x_forward_backward import TimeXForwardBackward
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jan 17 2021
@author: TG
'''
class Go_to_object_at_camera_sizeSM(Behavior):
	'''
	Go to an object so it occupy a certain percentage of the screen, while keeping it in the middle of the screen.
	'''


	def __init__(self):
		super(Go_to_object_at_camera_sizeSM, self).__init__()
		self.name = 'Go_to_object_at_camera_size'

		# parameters of this behavior
		self.add_parameter('object', 'person')
		self.add_parameter('object_score', 70)
		self.add_parameter('object_percent_size', 50)
		self.add_parameter('object_size_margins', 5)
		self.add_parameter('object_pixel_margins', 100)
		self.add_parameter('max_looking_time', 10)
		self.add_parameter('move_time_sec', 1)
		self.add_parameter('motor_throttle', 35)
		self.add_parameter('max_iteration', 240)

		# references to used behaviors
		self.add_behavior(Center_object_in_camera_middleSM, 'Center_object_in_camera_middle')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 919 165 
		# Look for the % of the screen the object occupies then sends an x movement outcome or finish if the % is within margins. This state fail if the object is lost or if it reaches the maximum iteration.

		# O 150 197 
		# The AUV centre the object in the middle of the camera screen

		# ! 1297 229 
		# The AUV centred the object in the middle of the camera and went to a distance where it occupies a certain % of the screen.

		# ! 31 491 
		# The AUV did not find the object or failed to change the angle.

		# O 716 384 
		# The AUV go backward then go back to the centring behaviour

		# O 720 42 
		# The AUV go forward then go back to the centring behaviour



	def create(self):
		# x:1251 y:243, x:321 y:493
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.iteration = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:151 y:231
			OperatableStateMachine.add('Center_object_in_camera_middle',
										self.use_behavior(Center_object_in_camera_middleSM, 'Center_object_in_camera_middle',
											parameters={'object': self.object, 'object_score': self.object_score, 'object_pixel_margins': self.object_pixel_margins, 'max_looking_time': self.max_looking_time}),
										transitions={'finished': 'Look_at_size', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:721 y:74
			OperatableStateMachine.add('Forward',
										TimeXForwardBackward(throttle=self.motor_throttle, moving_time=self.move_time_sec),
										transitions={'finished': 'Center_object_in_camera_middle'},
										autonomy={'finished': Autonomy.Off})

			# x:920 y:233
			OperatableStateMachine.add('Look_at_size',
										ObjectScreenPercent(searched_object=self.object, minimum_score=self.object_score, object_size_percent=self.object_percent_size, size_percent_range=self.object_size_margins, maximum_time=self.max_looking_time, max_iterations=self.max_iteration),
										transitions={'finished': 'finished', 'failed': 'failed', 'too_many_tries': 'failed', 'too_far': 'Forward', 'too_close': 'Backward'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off, 'too_many_tries': Autonomy.Off, 'too_far': Autonomy.Off, 'too_close': Autonomy.Off},
										remapping={'iterations_in': 'iteration', 'iterations_out': 'iteration'})

			# x:716 y:416
			OperatableStateMachine.add('Backward',
										TimeXForwardBackward(throttle=-self.motor_throttle, moving_time=self.move_time_sec),
										transitions={'finished': 'Center_object_in_camera_middle'},
										autonomy={'finished': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
