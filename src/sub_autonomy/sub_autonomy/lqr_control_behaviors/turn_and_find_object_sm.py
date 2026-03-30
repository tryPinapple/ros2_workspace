#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from camera_states.object_detection import ObjectDetection
from lqr_control_states.increment_yaw import IncrementYaw
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jan 17 2021
@author: Thomas Germain
'''
class Turn_and_find_objectSM(Behavior):
	'''
	Try to find an object with detectnet and after a certain time, if the object is not found turn the Yaw angle and look again.
	'''


	def __init__(self):
		super(Turn_and_find_objectSM, self).__init__()
		self.name = 'Turn_and_find_object'

		# parameters of this behavior
		self.add_parameter('object', 'person')
		self.add_parameter('object_score', 70)
		self.add_parameter('max_looking_time', 10)
		self.add_parameter('max_iteration', 16)
		self.add_parameter('yaw_turn_per_iteration', 45)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 658 216 
		# Change the Yaw angle then goes back to looking_for_object

		# O 328 48 
		# Look for a specified object, if not found fail and change the Yaw angle. This state fail it reaches the maximum iteration.

		# ! 110 408 
		# The AUV failed to change the angle or reached max iterations.

		# ! 699 103 
		# The object was found

		# O 537 161 
		# failed V

		# O 567 200 
		# finished A



	def create(self):
		time_to_move_sec = 5
		# x:656 y:102, x:359 y:411
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.iteration = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:329 y:96
			OperatableStateMachine.add('look_for_object',
										ObjectDetection(searched_object=self.object, minimum_score=self.object_score, search_time=self.max_looking_time, max_iterations=self.max_iteration),
										transitions={'finished': 'finished', 'failed': 'turn_right', 'too_many_tries': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off, 'too_many_tries': Autonomy.Off},
										remapping={'iterations_in': 'iteration', 'iterations_out': 'iteration'})

			# x:659 y:251
			OperatableStateMachine.add('turn_right',
										IncrementYaw(add_yaw_degree=self.yaw_turn_per_iteration, max_time=time_to_move_sec),
										transitions={'finished': 'look_for_object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
