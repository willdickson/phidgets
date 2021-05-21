#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('phidgets')
import rospy
import rosparam
import inspect

from std_msgs.msg import String

from phidgets.srv import SrvPhidgetsStepperFunction, SrvPhidgetsStepperFunctionResponse

import Phidgets
import Phidgets.Devices.Stepper


###############################################################################
###############################################################################
class PhidgetsStepper:

    def __init__(self):
        self.bInitialized = False
        self.iCount = 0
        self.bAttached = False
        
        # initialize
        rospy.init_node('phidgets_stepper', anonymous=True)
        
        # Load the parameters.
        self.params = rospy.get_param('phidgets_stepper', {})
        self.defaults = {}
        self.set_dict_with_preserve(self.params, self.defaults)
        rospy.set_param('phidgets_stepper', self.params)
        

        # Connect to the Phidget.
        self.stepper = Phidgets.Devices.Stepper.Stepper()
        self.stepper.openPhidget()
        self.stepper.setOnAttachHandler(self.attach_callback)
        self.stepper.setOnDetachHandler(self.detach_callback)

        # Subscriptions.        
        self.subCommand  = rospy.Subscriber('phidgets_stepper/command', String, self.command_callback)
        self.service = rospy.Service('phidgets_stepper', SrvPhidgetsStepperFunction, self.service_callback)
        rospy.sleep(1) # Allow time to connect publishers & subscribers.

        self.bInitialized = True
        
        
    def attach_callback(self, phidget):
        rospy.logwarn('PhidgetsStepper Attached to: %s, ID=%s' % (self.stepper.getDeviceName(), self.stepper.getDeviceID()))
        self.bAttached = True
        

    def detach_callback(self, phidget):
        rospy.logwarn ('PhidgetsStepper:  Detached.')
        self.bAttached = False


    # service_callback()
    # Handle the service request for a function call into the Phidgets library.
    #
    def service_callback(self, req):
        resp = SrvPhidgetsStepperFunctionResponse()
        
        if (self.bAttached):
            # Make the API call text.
            if (req.command in dir(self.stepper)):
                cmd = 'self.stepper.' + req.command + '('
                for i in range(len(req.args)):
                    cmd += str(req.args[i]) + ','
                cmd += ')'
                
                rospy.logwarn('Calling: %s' % cmd)
                rv = eval(cmd)
                if (type(rv)==type(True)):
                    resp = SrvPhidgetsStepperFunctionResponse(rvBool=rv)
                elif (type(rv)==type(0)):
                    resp = SrvPhidgetsStepperFunctionResponse(rvInt=rv)
                elif (type(rv)==type(3.14159)):
                    resp = SrvPhidgetsStepperFunctionResponse(rvFloat=rv)
                elif (type(rv)==type('asdf')):
                    resp = SrvPhidgetsStepperFunctionResponse(rvString=rv)
            else:
                rospy.logwarn('PhidgetsStepper does not have function: %s' % req.command)
        else:
            rospy.logwarn('PhidgetsStepper is not attached.')
            
        return resp
    
    
    # command_callback()
    # Handle messages sent to the command topic.
    #
    def command_callback(self, msg):
        if (msg.command == 'exit'):
            rospy.signal_shutdown('User requested exit.')


        elif (msg.command == 'help'):
            rospy.logwarn('The phidgets_stepper/command topic accepts the following string commands:')
            rospy.logwarn('    help     This message.')
            rospy.logwarn('    api      Print a list of PhidgetsStepper functions, with args.')
            rospy.logwarn('    doc      Print an expanded list of PhidgetsStepper commands, each with a description.')
            rospy.logwarn('    exit     Exit the program.')
            rospy.logwarn('')
            rospy.logwarn('You can send the above commands at the shell prompt via:')
            rospy.logwarn('rostopic pub -1 phidgets_stepper/command phidgets/MsgPhidgetsStepperCommand commandtext')
            rospy.logwarn('for example:')
            rospy.logwarn('    rostopic pub -1 phidgets_stepper/command phidgets/MsgPhidgetsStepperCommand help')
            rospy.logwarn('    rostopic pub -1 phidgets_stepper/command phidgets/MsgPhidgetsStepperCommand api')
            rospy.logwarn('    rostopic pub -1 phidgets_stepper/command phidgets/MsgPhidgetsStepperCommand doc')
            rospy.logwarn('    rostopic pub -1 phidgets_stepper/command phidgets/MsgPhidgetsStepperCommand exit')
            rospy.logwarn('')
            rospy.logwarn('')
            rospy.logwarn('Stepper function calls are provided via a ROS service, and may be called from the command-line:')
            rospy.logwarn('    rosservice call phidgets_stepper function [arg1,arg2,...]')
            rospy.logwarn('for example:')
            rospy.logwarn('    rosservice call phidgets_stepper getMotorCount []')
            rospy.logwarn('    rosservice call phidgets_stepper setEnabled [0,True]')
            rospy.logwarn('    rosservice call phidgets_stepper setAcceleration [0,100000]')
            rospy.logwarn('    rosservice call phidgets_stepper setTargetPosition [0,100000]')
            rospy.logwarn('    rosservice call phidgets_stepper setTargetPosition [0,0]')
            rospy.logwarn('')
            rospy.logwarn('Return values are in one of the fields of the service response structure, and which')
            rospy.logwarn('field depends on the data type.  You must know the returned type to know which field')
            rospy.logwarn('is the valid one.')
            rospy.logwarn('')


        elif (msg.command == 'api'):
            rospy.logwarn('')
            rospy.logwarn('The PhidgetStepper provides the following function calls:')
            for command in inspect.getmembers(self.stepper, inspect.ismethod):
                if (command[0][0] == '_'):
                    continue
                if (command[0] in ['closePhidget',
                                   'getServerAddress',
                                   'getServerID',
                                   'isAttachedToServer',
                                   'openPhidget',
                                   'openRemote',
                                   'openRemoteIP',
                                   'setOnAttachHandler',
                                   'setOnDetachHandler',
                                   'setOnErrorhandler',
                                   'setOnErrorHandler',
                                   'setOnCurrentChangeHandler',
                                   'setOnInputChangeHandler',
                                   'setOnPositionChangeHandler',
                                   'setOnServerConnectHandler',
                                   'setOnServerDisconnectHandler',
                                   'setOnVelocityChangeHandler',
                                   'waitForAttach']):
                    continue
                
                (args, varargs, keywords, defaults) = inspect.getargspec(command[1])
                prototype = '%s(' % command[0]
                if (args is not None):
                    for arg in args[1:]:
                        prototype += '%s, ' % arg
                if (varargs is not None):
                    for arg in varargs:
                        prototype += '%s, ' % arg
                if (keywords is not None):
                    for arg in keywords:
                        prototype += '%s=, ' % arg
                
                prototype += ')'
                rospy.logwarn(prototype)
                    
            rospy.logwarn('')
            rospy.logwarn('')
            rospy.logwarn('Stepper function calls are provided via a ROS service, and may be called from the command-line:')
            rospy.logwarn('    rosservice call phidgets_stepper function [arg1,arg2,...]')
            rospy.logwarn('for example:')
            rospy.logwarn('    rosservice call phidgets_stepper getMotorCount []')
            rospy.logwarn('    rosservice call phidgets_stepper setCurrentPosition [0,0]')
            rospy.logwarn('    rosservice call phidgets_stepper setEngaged [0,True]')
            rospy.logwarn('    rosservice call phidgets_stepper setAcceleration [0,100000]')
            rospy.logwarn('    rosservice call phidgets_stepper setVelocityLimit [0,10000]')
            rospy.logwarn('    rosservice call phidgets_stepper setTargetPosition [0,100000]')
            rospy.logwarn('')
                    

        elif (msg.command == 'doc'):
            rospy.logwarn('')
            rospy.logwarn('Documentation for the PhidgetStepper API:')
            for command in inspect.getmembers(self.stepper, inspect.ismethod):
                if (command[0][0] != '_'):
                    rospy.logwarn('  %s' % command[0])
                    cmd = 'self.stepper.' + command[0] + '.__doc__'
                    rospy.logwarn(eval(cmd))
                    
            rospy.logwarn('')

        else:
            rospy.logwarn('Unknown command: %s' % msg.command)
        
        
    
    
        
    # set_dict(self, dTarget, dSource, bPreserve)
    # Takes a target dictionary, and enters values from the source dictionary, overwriting or not, as asked.
    # For example,
    #    dT={'a':1, 'b':2}
    #    dS={'a':0, 'c':0}
    #    Set(dT, dS, True)
    #    dT is {'a':1, 'b':2, 'c':0}
    #
    #    dT={'a':1, 'b':2}
    #    dS={'a':0, 'c':0}
    #    Set(dT, dS, False)
    #    dT is {'a':0, 'b':2, 'c':0}
    #
    def set_dict(self, dTarget, dSource, bPreserve):
        for k,v in dSource.iteritems():
            bKeyExists = (k in dTarget)
            if (not bKeyExists) and type(v)==type({}):
                dTarget[k] = {}
            if ((not bKeyExists) or not bPreserve) and (type(v)!=type({})):
                dTarget[k] = v
                    
            if type(v)==type({}):
                self.set_dict(dTarget[k], v, bPreserve)
    
    
    def set_dict_with_preserve(self, dTarget, dSource):
        self.set_dict(dTarget, dSource, True)
    
    def set_dict_with_overwrite(self, dTarget, dSource):
        self.set_dict(dTarget, dSource, False)




    def run(self):
        rospy.spin()


if __name__ == '__main__':

    ps = PhidgetsStepper()
    ps.run()

