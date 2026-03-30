#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from lqr_control_states.increment_x import IncrementX
from lqr_control_states.increment_y import IncrementY
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 23 2023
@author: cgirard
'''
class carrouselSM(Behavior):
	'''
	Move 1m forward, 1m left, 1m backward, then 1m right, with pauses of 5s inbetween.
	'''


	def __init__(self):
		super(carrouselSM, self).__init__()
		self.name = 'carrousel'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1304 y:33, x:1205 y:296
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:53 y:269
			OperatableStateMachine.add('Stay',
										IncrementX(add_x_distance=0.001, max_time=5),
										transitions={'reached': 'wait0', 'failed': 'wait0'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:737 y:39
			OperatableStateMachine.add('backward',
										IncrementX(add_x_distance=-1, max_time=10),
										transitions={'reached': 'wait3', 'failed': 'wait3'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:69 y:37
			OperatableStateMachine.add('forward',
										IncrementX(add_x_distance=1, max_time=10),
										transitions={'reached': 'wait1', 'failed': 'wait1'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:360 y:41
			OperatableStateMachine.add('left',
										IncrementY(add_y_distance=-1, max_time=10),
										transitions={'reached': 'wait2', 'failed': 'wait2'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1091 y:36
			OperatableStateMachine.add('right',
										IncrementY(add_y_distance=1, max_time=10),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:63 y:156
			OperatableStateMachine.add('wait0',
										WaitState(wait_time=5),
										transitions={'done': 'forward'},
										autonomy={'done': Autonomy.Off})

			# x:226 y:39
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=5),
										transitions={'done': 'left'},
										autonomy={'done': Autonomy.Off})

			# x:543 y:37
			OperatableStateMachine.add('wait2',
										WaitState(wait_time=5),
										transitions={'done': 'backward'},
										autonomy={'done': Autonomy.Off})

			# x:915 y:37
			OperatableStateMachine.add('wait3',
										WaitState(wait_time=5),
										transitions={'done': 'right'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
