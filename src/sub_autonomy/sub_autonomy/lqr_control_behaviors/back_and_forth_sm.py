#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lqr_control_states.increment_x import IncrementX
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 23 2023
@author: Troll
'''
class back_and_forthSM(Behavior):
	'''
	advance 10m then come back 10m
	'''


	def __init__(self):
		super(back_and_forthSM, self).__init__()
		self.name = 'back_and_forth'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:31 y:352, x:11 y:288
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:6 y:58
			OperatableStateMachine.add('MoveToMeter1',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter2', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:56 y:143
			OperatableStateMachine.add('MoveBackToMeter1',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter0', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:238 y:146
			OperatableStateMachine.add('MoveBackToMeter2',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter1', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:379 y:146
			OperatableStateMachine.add('MoveBackToMeter3',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter2', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:522 y:144
			OperatableStateMachine.add('MoveBackToMeter4',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter3', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:664 y:149
			OperatableStateMachine.add('MoveBackToMeter5',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter4', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:807 y:151
			OperatableStateMachine.add('MoveBackToMeter6',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter5', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:941 y:150
			OperatableStateMachine.add('MoveBackToMeter7',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter6', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:1083 y:153
			OperatableStateMachine.add('MoveBackToMeter8',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter7', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:1247 y:150
			OperatableStateMachine.add('MoveBackToMeter9',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'MoveBackToMeter8', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:1154 y:55
			OperatableStateMachine.add('MoveToMeter10',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveBackToMeter9', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:135 y:58
			OperatableStateMachine.add('MoveToMeter2',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter3', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:263 y:56
			OperatableStateMachine.add('MoveToMeter3',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter4', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:390 y:55
			OperatableStateMachine.add('MoveToMeter4',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter5', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:519 y:55
			OperatableStateMachine.add('MoveToMeter5',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter6', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:644 y:54
			OperatableStateMachine.add('MoveToMeter6',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter7', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:775 y:56
			OperatableStateMachine.add('MoveToMeter7',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter8', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:901 y:57
			OperatableStateMachine.add('MoveToMeter8',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter9', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:1028 y:59
			OperatableStateMachine.add('MoveToMeter9',
										IncrementX(add_x_distance=1, max_time=30),
										transitions={'reached': 'MoveToMeter10', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})

			# x:119 y:228
			OperatableStateMachine.add('MoveBackToMeter0',
										IncrementX(add_x_distance=-1, max_time=30),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Full, 'failed': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
