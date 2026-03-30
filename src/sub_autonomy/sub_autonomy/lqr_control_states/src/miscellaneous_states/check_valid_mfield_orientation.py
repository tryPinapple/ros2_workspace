#!/usr/bin/env python
from flexbe_core import EventState, Logger


class CheckValidMField(EventState):
    '''
    Implements a state that checks if the given condition is true based on multiple userdata inputs
    provided as a list to the calculation function and returns the corresponding outcome.
    This state can be used if the further control flow of the behavior depends on a simple condition.

    -- predicate    function    The condition whose truth value will be evaluated.
                                It could be a private function (self.foo) manually defined in a behavior's source code
                                or a lambda function (e.g., lambda x: x[0]^2 + x[1]^2).
    -- input_keys   string[]    List of available input keys.

    ># input_keys   object[]    Input(s) to the calculation function as a list of userdata.
                                The individual inputs can be accessed as list elements (see lambda expression example).

    <= true                     Returned if the condition evaluates to True
    <= false                    Returned if the condition evaluates to False
    '''

    def __init__(self):
        '''Constructor'''
        super(CheckValidMField, self).__init__(outcomes=['valid', 'incorrect'],
                                                          input_keys=['x_mfield', 'old_x_mfield'])
        self._outcome = 'incorrect'

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
        if userdata.x_mfield is not None:
             
             Logger.logwarn('Value received, validating...')
             if userdata.x_mfield > 0 and userdata.old_x_mfield > 0:
                 diff = userdata.x_mfield - userdata.old_x_mfield
                 Logger.loginfo('diff is equal to %f' % diff)
                 if diff < 0:
                     self._outcome = 'valid'

                 else:
                     self._outcome = 'incorrect'

                     
        else:
            Logger.logwarn('no value passed, retry')
