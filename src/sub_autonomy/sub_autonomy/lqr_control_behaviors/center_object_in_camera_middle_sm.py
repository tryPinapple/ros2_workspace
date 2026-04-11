#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from camera_states.object_alignment import ObjectAlignment
from lqr_control_states.increment_pitch import IncrementPitch
from lqr_control_states.increment_yaw import IncrementYaw
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jan 17 2021
@author: Thomas Germain
'''
class Center_object_in_camera_middleSM(Behavior):
	'''
	Centre an object in the middle of the camera screen by using the Yaw and Pitch angles.
	'''


	def __init__(self):
		super(Center_object_in_camera_middleSM, self).__init__()
		self.name = 'Center_object_in_camera_middle'

		# parameters of this behavior
		self.add_parameter('object', 'person')
		self.add_parameter('object_score', 70)
		self.add_parameter('object_pixel_margins', 100)
		self.add_parameter('max_looking_time', 10)
		self.add_parameter('degree_change', 3)
		self.add_parameter('max_iteration', 100)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# ! 1378 758 
		# Any resemblance with Soron's eye is only due to your imagination....maybe....

		# O 96 298 
		# Decrease the pitch angle of the AUV

		# O 870 295 
		# Decrease the yaw angle of the AUV

		# O 445 291 
		# Increase the pitch angle of the AUV

		# O 1212 296 
		# Increase the yaw angle of the AUV

		# O 606 23 
		# Look for an object and send directional outcome depending on the position of the object centre  to make it coincide with the camera centre. This state fail if the object is lost or if it reaches the maximum iteration.

		# ! 1346 42 
		# The object has been centred within the acceptable margin of the camera centre

		# ! 609 569 
		# The object was not found or the AUV wasn't able to change the angle

		# O 854 237 
		# finished A

		# O 1039 239 
		# finished A

		# O 447 238 
		# finished A

		# O 617 237 
		# finished A

		# O 475 197 
		# object_down V

		# O 845 191 
		# object_left V

		# O 999 193 
		# object_right V

		# O 632 186 
		# object_up V



	def create(self):
		time_to_move_sec = 5
		# x:1299 y:49, x:734 y:541
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.iteration = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:678 y:95
			OperatableStateMachine.add('center_object_direction',
										ObjectAlignment(searched_object=self.object, minimum_score=self.object_score, center_precision=self.object_pixel_margins, max_align_time=self.max_looking_time, max_iterations=self.max_iteration),
										transitions={'finished': 'finished', 'failed': 'failed', 'too_many_tries': 'failed', 'object_up': 'turn_pitch_up', 'object_down': 'turn_pitch_down', 'object_left': 'turn_yaw_left', 'object_right': 'turn_yaw_right'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off, 'too_many_tries': Autonomy.Off, 'object_up': Autonomy.Off, 'object_down': Autonomy.Off, 'object_left': Autonomy.Off, 'object_right': Autonomy.Off},
										remapping={'iterations_in': 'iteration', 'iterations_out': 'iteration'})

			# x:98 y:320
			OperatableStateMachine.add('turn_pitch_down',
										IncrementPitch(add_pitch_degree=-self.degree_change, max_time=time_to_move_sec),
										transitions={'finished': 'center_object_direction', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:445 y:316
			OperatableStateMachine.add('turn_pitch_up',
										IncrementPitch(add_pitch_degree=self.degree_change, max_time=time_to_move_sec),
										transitions={'finished': 'center_object_direction', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:877 y:317
			OperatableStateMachine.add('turn_yaw_left',
										IncrementYaw(add_yaw_degree=-self.degree_change, max_time=time_to_move_sec),
										transitions={'finished': 'center_object_direction', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1213 y:320
			OperatableStateMachine.add('turn_yaw_right',
										IncrementYaw(add_yaw_degree=self.degree_change, max_time=time_to_move_sec),
										transitions={'finished': 'center_object_direction', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
