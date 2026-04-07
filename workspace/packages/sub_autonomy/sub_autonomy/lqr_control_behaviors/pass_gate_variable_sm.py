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
from lqr_control_states.increment_yaw import IncrementYaw
from lqr_control_states.increment_z import IncrementZ
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 03 2023
@author: Etienne Pellerin
'''
class pass_gate_variableSM(Behavior):
	'''
	incrementation fo x degree to pass trought the gate
	'''


	def __init__(self):
		super(pass_gate_variableSM, self).__init__()
		self.name = 'pass_gate_variable'

		# parameters of this behavior
		self.add_parameter('turn_angle', 0)
		self.add_parameter('step_distance', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		depth = 0.2
		# x:895 y:330, x:86 y:362
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:68 y:246
			OperatableStateMachine.add('go_down',
										IncrementZ(add_z_distance=depth, max_time=5),
										transitions={'reached': 'wait0', 'failed': 'wait0'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:526 y:19
			OperatableStateMachine.add('move1',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait2', 'failed': 'wait2'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:751 y:230
			OperatableStateMachine.add('move10',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait11', 'failed': 'wait11'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:678 y:24
			OperatableStateMachine.add('move11',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait12', 'failed': 'wait12'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:859 y:164
			OperatableStateMachine.add('move12',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'finished', 'failed': 'finished'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:525 y:118
			OperatableStateMachine.add('move2',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait3', 'failed': 'wait3'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:524 y:238
			OperatableStateMachine.add('move3',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait4', 'failed': 'wait4'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:519 y:355
			OperatableStateMachine.add('move4',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait5', 'failed': 'wait5'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:513 y:472
			OperatableStateMachine.add('move5',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait6', 'failed': 'wait6'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:449 y:588
			OperatableStateMachine.add('move6',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait7', 'failed': 'wait7'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:779 y:571
			OperatableStateMachine.add('move7',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait8', 'failed': 'wait8'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:778 y:467
			OperatableStateMachine.add('move8',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait9', 'failed': 'wait9'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:757 y:353
			OperatableStateMachine.add('move9',
										IncrementX(add_x_distance=self.step_distance, max_time=5),
										transitions={'reached': 'wait10', 'failed': 'wait10'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:142 y:24
			OperatableStateMachine.add('turn right',
										IncrementYaw(add_yaw_degree=self.turn_angle, max_time=5),
										transitions={'finished': 'wait1', 'failed': 'wait1'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:110 y:142
			OperatableStateMachine.add('wait0',
										WaitState(wait_time=10),
										transitions={'done': 'turn right'},
										autonomy={'done': Autonomy.Off})

			# x:384 y:21
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'move1'},
										autonomy={'done': Autonomy.Off})

			# x:630 y:297
			OperatableStateMachine.add('wait10',
										WaitState(wait_time=1),
										transitions={'done': 'move10'},
										autonomy={'done': Autonomy.Off})

			# x:642 y:140
			OperatableStateMachine.add('wait11',
										WaitState(wait_time=1),
										transitions={'done': 'move11'},
										autonomy={'done': Autonomy.Off})

			# x:859 y:47
			OperatableStateMachine.add('wait12',
										WaitState(wait_time=1),
										transitions={'done': 'move12'},
										autonomy={'done': Autonomy.Off})

			# x:381 y:92
			OperatableStateMachine.add('wait2',
										WaitState(wait_time=1),
										transitions={'done': 'move2'},
										autonomy={'done': Autonomy.Off})

			# x:379 y:193
			OperatableStateMachine.add('wait3',
										WaitState(wait_time=1),
										transitions={'done': 'move3'},
										autonomy={'done': Autonomy.Off})

			# x:378 y:292
			OperatableStateMachine.add('wait4',
										WaitState(wait_time=1),
										transitions={'done': 'move4'},
										autonomy={'done': Autonomy.Off})

			# x:372 y:391
			OperatableStateMachine.add('wait5',
										WaitState(wait_time=1),
										transitions={'done': 'move5'},
										autonomy={'done': Autonomy.Off})

			# x:369 y:482
			OperatableStateMachine.add('wait6',
										WaitState(wait_time=1),
										transitions={'done': 'move6'},
										autonomy={'done': Autonomy.Off})

			# x:632 y:613
			OperatableStateMachine.add('wait7',
										WaitState(wait_time=1),
										transitions={'done': 'move7'},
										autonomy={'done': Autonomy.Off})

			# x:637 y:502
			OperatableStateMachine.add('wait8',
										WaitState(wait_time=1),
										transitions={'done': 'move8'},
										autonomy={'done': Autonomy.Off})

			# x:635 y:403
			OperatableStateMachine.add('wait9',
										WaitState(wait_time=1),
										transitions={'done': 'move9'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
