#!/usr/bin/env python
# demo_client_node.py, part of ROS package mekamon_remote.
# Demo of interacting with base_controller node in mekamon_base package on Pi.
# Wes Freeman, Reach Robotics 2018
#
# A lot of this code is just for the test GUI.

import rospy
from std_msgs.msg import * # standard ROS msg types: int, float etc.
from geometry_msgs.msg import Twist # standard ROS msg used for Transform mode
from mekamon_msgs.msg import * # custom message & service types
from mekamon_msgs.srv import * # custom message & service types
from appJar import gui # simple manual controls

# storage for values reported by the robot base_controller
class RobotState():

    batt_milliamps = 0 # 16b unsigned
    batt_millivolts = 0
    
    joints_FLK = 0 # 8b unsigned
    joints_FLT = 0
    joints_FLH = 0
    
    joints_FRK = 0
    joints_FRT = 0
    joints_FRH = 0
    
    joints_BLK = 0
    joints_BLT = 0
    joints_BLH = 0
    
    joints_BRK = 0
    joints_BRT = 0
    joints_BRH = 0
    
    heading = 0 # 16b unsigned
    
# storage for values set by this node, to be published to the robot.
class RobotStateNew(): 

    kinematicMode = 3 # default mode is startup mode, 
    # compatible with Transform & Gait commands.
    # This is in here while others aren't because it is needed by another 
    # function - ROSLoop()
       
    joints_FLK = 600 # initial values set to middle of range.
    joints_FLT = 400
    joints_FLH = 600
    
    joints_FRK = 600
    joints_FRT = 400
    joints_FRH = 600
    
    joints_BLK = 600
    joints_BLT = 400
    joints_BLH = 600
    
    joints_BRK = 600
    joints_BRT = 400
    joints_BRH = 600       

    transform_strafe = 0.0 # float -1...0...+1
    transform_fwd = 0.0
    transform_turn = 0.0


###################      ROS SUBSCRIBER CALLBACKS   ###############################

# Battery report rate is set by the base_controller node.
# It should begin publishing automatically.
# The rate can be changed via ROS param (launchfile).
def batteryCallback(msg):
    # store battery values from the message
    RobotState.batt_milliamps = msg.milliamps
    RobotState.batt_millivolts = msg.millivolts
    
    # update the GUI
    string = "batt mV: " + str(RobotState.batt_millivolts)
    app.setLabel("l_batt_mv", string)
    string = "batt mA: " + str(RobotState.batt_milliamps) 
    app.setLabel("l_batt_ma", string)      


# Joint angles report rate is set to 0 (off) by default.
# it can be set by ROS param (launchfile)
# or by the ROS service SetJointReportRate.
def jointsCallback(msg):
    # store joint values from the message. 
    RobotState.joints_FLK = msg.front_left_knee  
    RobotState.joints_FLT = msg.front_left_thigh 
    RobotState.joints_FLH = msg.front_left_hip   
    
    RobotState.joints_FRK = msg.front_right_knee  
    RobotState.joints_FRT = msg.front_right_thigh 
    RobotState.joints_FRH = msg.front_right_hip   
    
    RobotState.joints_BLK = msg.back_left_knee  
    RobotState.joints_BLT = msg.back_left_thigh 
    RobotState.joints_BLH = msg.back_left_hip   
    
    RobotState.joints_BRK = msg.back_right_knee  
    RobotState.joints_BRT = msg.back_right_thigh 
    RobotState.joints_BRH = msg.back_right_hip   

    # update the GUI
    string = "joints_FLK: " + str(RobotState.joints_FLK)
    app.setLabel("l_joints_FLK", string)
    string = "joints_FLT: " + str(RobotState.joints_FLT) 
    app.setLabel("l_joints_FLT", string)
    string = "joints_FLH: " + str(RobotState.joints_FLH)
    app.setLabel("l_joints_FLH", string)

    string = "joints_FRK: " + str(RobotState.joints_FRK)
    app.setLabel("l_joints_FRK", string)
    string = "joints_FRT: " + str(RobotState.joints_FRT) 
    app.setLabel("l_joints_FRT", string)
    string = "joints_FRH: " + str(RobotState.joints_FRH)
    app.setLabel("l_joints_FRH", string)    
  
    string = "joints_BLK: " + str(RobotState.joints_BLK)
    app.setLabel("l_joints_BLK", string)
    string = "joints_BLT: " + str(RobotState.joints_BLT) 
    app.setLabel("l_joints_BLT", string)
    string = "joints_BLH: " + str(RobotState.joints_BLH)
    app.setLabel("l_joints_BLH", string)

    string = "joints_BRK: " + str(RobotState.joints_BRK)
    app.setLabel("l_joints_BRK", string)
    string = "joints_BRT: " + str(RobotState.joints_BRT) 
    app.setLabel("l_joints_BRT", string)
    string = "joints_BRH: " + str(RobotState.joints_BRH)
    app.setLabel("l_joints_BRH", string)  



# Compass heading report rate is set to 0 (off) by default.
# it can be set by ROS param (launchfile)
# or by the ROS service SetHeadingReportRate.
def headingCallback(msg):
    # store heading value from the message
    RobotState.heading = msg.data
    
    # update the GUI
    string = "Deg: " + str(RobotState.heading)
    app.setLabel("l_heading", string)

    
#####################     ROS PUBLISHERS   ###################################

# ROS publishers are designed for more-or-less continuous streams of a commands or data.

# Separate the publisher rate from the GUI update rate by having the GUI
# write to the RobotStateNew() container class,
# and let the main loop publish at the rate it wants to.
    
def jointAnglesPublisher(jointsPub):
    # Create ROS message container.
    jointState = JointAngles()
    # This is a custom msg for Mekamon, uses 12 x uint16.
    # 0-1024 range which represents almost a full 360 turn.
    # The actual range of the joints is approx:
    # knee: min 400 (extended) max 800 (folded)
    # thigh: min 200 (down) max 600 (up)
    # hip: min 400 (CW from above) max 800 (CCW)    
    
    # populate it with our desired values
    jointState.front_left_knee = RobotStateNew.joints_FLK
    jointState.front_left_thigh = RobotStateNew.joints_FLT
    jointState.front_left_hip = RobotStateNew.joints_FLH 
    
    jointState.front_right_knee = RobotStateNew.joints_FRK 
    jointState.front_right_thigh = RobotStateNew.joints_FRT 
    jointState.front_right_hip = RobotStateNew.joints_FRH 
    
    jointState.back_left_knee = RobotStateNew.joints_BLK 
    jointState.back_left_thigh = RobotStateNew.joints_BLT 
    jointState.back_left_hip = RobotStateNew.joints_BLH 
    
    jointState.back_right_knee = RobotStateNew.joints_BRK 
    jointState.back_right_thigh = RobotStateNew.joints_BRT 
    jointState.back_right_hip = RobotStateNew.joints_BRH
    
    # publish the message to ROS network
    jointsPub.publish(jointState)

def twistPublisher(twistPub):
    # create ROS message container.
    vels = Twist()    
    # (standard ROS msg, see http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html )
    # Uses 2 * vector of 3 * float, for 6 DOF but we only need 3 DOF; 2 linear, 1 rotation.
    # The floats should be -1 to +1.
    # They are proportional and do not directly represent velocities.
    
    # populate it with our desired values
    vels.linear.x = RobotStateNew.transform_strafe
    vels.linear.y = RobotStateNew.transform_fwd
    vels.angular.z = RobotStateNew.transform_turn

    # publish the message to ROS network
    twistPub.publish(vels)        

    
##########################    ROS SERVICE CLIENTS  #########################################

# ROS services are designed for one-off or sporadic commands and requests.

# This service takes a 16b unsigned int representing milliseconds between 
# joint angle reports from the robot.
def clientSetJointReportRateSvc(interval):
    # wait for service availability
    rospy.wait_for_service('set_joint_report_rate')
    try: 
        # create a handle for calling the service:
        setJointReportRateSvc = rospy.ServiceProxy('set_joint_report_rate', SetJointReportRate)
        # can now call the service like a function
        setJointReportRateSvc(interval) 
        # (ignoring response value from this particular service)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# This service takes a 16b unsigned int representing milliseconds between 
# compass heading reports from the robot.
def clientSetHeadingReportRateSvc(interval):
    # wait for service availability
    rospy.wait_for_service('set_heading_report_rate')
    try: 
        # create a handle for calling the service:
        setHeadingReportRateSvc = rospy.ServiceProxy('set_heading_report_rate', SetHeadingReportRate)
        # can now call the service like a function
        setHeadingReportRateSvc(interval) 
        # (ignoring response value from this particular service)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e        

# This service takes an 8b unsigned int per leg to set compliance.
# Currently the continuous range isn't implemented, it's either stiff (0) or loose (255).
def clientSetLegComplianceSvc(FL, FR, BL, BR):
    # wait for service availability
    rospy.wait_for_service('set_leg_compliance')
    try: 
        # create a handle for calling the service:
        setLegComplianceSvc = rospy.ServiceProxy('set_leg_compliance', SetLegCompliance)
        # can now call the service like a function
        setLegComplianceSvc(FL, FR, BL, BR) 
        # (ignoring response value from this service)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# This service takes uint8 per R,G,B
def clientSetHeadColourSvc(red, green, blue):
    # wait for service availability
    rospy.wait_for_service('set_head_colour')
    try: 
        # create a handle for calling the service:
        setHeadColourSvc = rospy.ServiceProxy('set_head_colour', SetHeadColour)
        # can now call the service like a function
        setHeadColourSvc(red, green, blue) 
        # (ignoring response value from this service)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# This service takes 2 uint8's, for parameter and value.
# There are 8 different automatic IK & FK gait settings controlled by this service.
# They are only relevant in Kinematic mode, with Transform commands.
#
#       param                  value interpreted as
# 0 - GAIT_STANCE_ANGLE,    Hip rotation offset angle +- 45, default 0. +ve is narrow, -ve is wide
# 1 - GAIT_STANCE_DISTANCE  Distance in mm from hip axis to foot. 40-200, default 110
# 2 - GAIT_WALKING_SPEED    Hip rotation degrees per second /4. 0-600, default 328 (value 0-150, def 82)
# 3 - GAIT_STEP_DURATION    percentage of 90 deg hip motion that leg is lifted. 15-90, default 56.
# 4 - GAIT_STEP_SHIFT       percentage of 90 deg hip motion that body is shifted?? 15-90, default 71.
# 5 - GAIT_STEP_HEIGHT      height in mm. 0-120, default 90.
# 6 - GAIT_STEPS            n-legged gait - accepts only 2 for 'TROT' or 4 for 'CRAWL' 
# 7 - GAIT_BODY_HEIGHT      height in mm when standing. 1-120, default 36.
def clientSetGaitParamSvc(param, value):
    # wait for service availability
    rospy.wait_for_service('set_gait_param')
    try: 
        # create a handle for calling the service:
        setGaitParamSvc = rospy.ServiceProxy('set_gait_param', SetGaitParam)
        # can now call the service like a function
        setGaitParamSvc(param, value) 
        # (ignoring response value from this service)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# This service takes single uint8 for kinematic mode. Options are:
# 0 - 'MOVE_GYRATE' - preset test animation
# 1 - 'MOVE_MIMIC' - all logs copy manual movement of front-left leg. 
# 2 - 'MOVE_EGYPTIAN' - preset test animation
# 3 - 'MOVE_KINEMATIC' - default used for Transform + gait params mode
# 4 - 'MOVE_PUPPETEER_COORDINATE' - required for foot-coordinates mode - NOT IMPLEMENTED
# 5 - 'MOVE_PUPPETEER_JOINT_ANGLE' - required for joint angles mode
def clientSetKinematicModeSvc():
    # wait for service availability
    rospy.wait_for_service('set_kinematic_mode')
    try: 
        # create a handle for calling the service:
        setKinematicModeSvc = rospy.ServiceProxy('set_kinematic_mode', SetKinematicMode)
        # can now call the service like a function, passing the value from robot state
        setKinematicModeSvc(RobotStateNew.kinematicMode) 
        # (ignoring response value from this service)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
        
####################   GUI FUNCS FOR ROS SERVICES   ####################################
    
def guiSetJointReportRate():
    # get value from GUI input box. It will be float so cast to int
    interval = int(app.getEntry("jointReportRate"))
    # check it's a sane value. Milliseconds, 16 bit unsigned int.
    # robot internal rate is 50Hz / 20ms 
    # rate 0 = off.
    if (interval < 20): interval = 0
    elif (interval > 65535): interval = 65535
    # call the ROS service client 
    clientSetJointReportRateSvc(interval)

def guiSetHeadingReportRate():
    # get value from GUI input box. It will be float so cast to int
    interval = int(app.getEntry("headingReportRate"))
    # check it's a sane value. Milliseconds, 16 bit unsigned int.
    # Robot internal rate is 200Hz / 5ms, though comms may not run that fast
    # rate 0 = off.
    if (interval < 5): interval = 0
    elif (interval > 65535): interval = 65535
    # call the ROS service client 
    clientSetHeadingReportRateSvc(interval)
    
def guiSetLegCompliance():
    # Get values from GUI, tickboxes return booleans. The robot currently only 
    # accepts on or off not degrees of compliance.
    if (app.getCheckBox("complianceFL")): FL = 255
    else: FL = 0
    if (app.getCheckBox("complianceFR")): FR = 255
    else: FR = 0
    if (app.getCheckBox("complianceBL")): BL = 255
    else: BL = 0
    if (app.getCheckBox("complianceBR")): BR = 255
    else: BR = 0    
    
    # call the ROS service client 
    clientSetLegComplianceSvc(FL, FR, BL, BR)

def guiSetHeadColour():
    # get values from GUI
    red = app.getScale("Red")
    green = app.getScale("Green")   
    blue = app.getScale("Blue")     

    # call the ROS service client 
    clientSetHeadColourSvc(red, green, blue)

def guiSetKinematicMode():
    # get value from GUI radio button, which will be a string
    kinModeStr = app.getRadioButton("kinematic")   
    # convert and write value to storage for new robot state params
    RobotStateNew.kinematicMode = int(kinModeStr)
    # call the ROS service client 
    clientSetKinematicModeSvc()

# gait param service encompasses 8 different settings which need their own funcs
def guiSetGaitParamStanceAngle():
    # get value from GUI
    value = app.getScale("StanceAngle")
    # call the ROS service client 
    clientSetGaitParamSvc(0, value)
    
def guiSetGaitParamStanceDistance():
    # get value from GUI
    value = app.getScale("StanceDistance")
    # call the ROS service client 
    clientSetGaitParamSvc(1, value)    

def guiSetGaitParamWalkingSpeed():
    # get value from GUI
    # NOTE this is scaled to fit in 8bits. The robot will scale it back up at it's end
    value = app.getScale("WalkingSpeed") /4
    # call the ROS service client 
    clientSetGaitParamSvc(2, value)

def guiSetGaitParamStepDuration():
    # get value from GUI
    value = app.getScale("StepDuration")    
    # call the ROS service client 
    clientSetGaitParamSvc(3, value)

def guiSetGaitParamStepShift():
    # get value from GUI
    value = app.getScale("StepShift")        
    # call the ROS service client 
    clientSetGaitParamSvc(4, value)
    
def guiSetGaitParamStepHeight():
    # get value from GUI
    value = app.getScale("StepHeight")        
    # call the ROS service client 
    clientSetGaitParamSvc(5, value)

def guiSetGaitParamSteps():
    # get value from GUI
    stepsString = app.getRadioButton("r_steps")
    if (stepsString == "Crawl"): value = 4
    else: value = 2
    # call the ROS service client 
    clientSetGaitParamSvc(6, value)

def guiSetGaitParamBodyHeight():
    # get value from GUI
    value = app.getScale("BodyHeight")        
    if (value < 1): value = 1 # shouldn't be zero.
    # call the ROS service client 
    clientSetGaitParamSvc(7, value)     
    
    
####################   GUI FUNCS FOR ROS PUBLISHERS  ####################################

# these just put the values in RobotStateNew, they don't do the actual publishing.

def guiTransform():
    # get values from GUI, they are ints +- 80 but need to be floats -1.0 to +1.0
    RobotStateNew.transform_strafe = app.getScale("Strafe") * 0.0125
    RobotStateNew.transform_fwd = app.getScale("Forward") * 0.0125  
    RobotStateNew.transform_turn = app.getScale("Turn") * 0.0125

def guiJointControl():
    # get values from GUI
    RobotStateNew.joints_FLK = app.getScale("FLK") 
    RobotStateNew.joints_FLT = app.getScale("FLT") 
    RobotStateNew.joints_FLH = app.getScale("FLH") 

    RobotStateNew.joints_FRK = app.getScale("FRK")  
    RobotStateNew.joints_FRT = app.getScale("FRT") 
    RobotStateNew.joints_FRH = app.getScale("FRH") 

    RobotStateNew.joints_BLK = app.getScale("BLK")  
    RobotStateNew.joints_BLT = app.getScale("BLT") 
    RobotStateNew.joints_BLH = app.getScale("BLH") 

    RobotStateNew.joints_BRK = app.getScale("BRK")  
    RobotStateNew.joints_BRT = app.getScale("BRT") 
    RobotStateNew.joints_BRH = app.getScale("BRH")         
    
def guiTransformStop():
    # reset the sliders
    app.setScale("Strafe", 0, callFunction=False)
    app.setScale("Forward", 0, callFunction=False)
    app.setScale("Turn", 0, callFunction=False)
    # now call the func to change RobotStateNew
    guiTransform()
    # and the next publisher loop should actually stop the robot.
    

###############################################################################

def ROSLoop(jointsPub, twistPub):

    #loop speed in Hz. May want it faster
    loopRate = rospy.Rate(10) 

    while not rospy.is_shutdown():

        # (checking for subscriber callbacks happens without a ros::spin in Python)

        # publish our desired robot state to ROS topics, depending on Kinematic Mode.
        kinMode = RobotStateNew.kinematicMode
        if(kinMode == 3):
            twistPublisher(twistPub)
        #if(kinMode == 4):
            # do nothing - this mode not implemented
        if(kinMode == 5):
            jointAnglesPublisher(jointsPub)
        
        loopRate.sleep()
        
def guiShutdown(null):
    app.stop() # shut down the GUI otherwise user has to close it.

###############################################################################


def main():

    # Set up GUI widgets 
    
    app.setFont(10) # seems like font sizes can only be set once
    app.setSticky("") # turn off widget expansion in general  

    # split GUI into 'frames' to organise it 
    app.startFrame("MAIN", row=0, column=0)
    
    # ------------------- Kinematic Mode ------------------------------------------
    
    # section title    
    app.addLabel("t_kinematic", "Kinematic Mode", 0, 0, 2)
    # use radio button as only one mode can be active
    # adding labels as it's easier to have numeric values returned from the button itself
    app.addLabel("l_gyrate", "Gyrate",   1, 0)
    app.addRadioButton("kinematic", "0", 1, 1)
    
    app.addLabel("l_mimic", "Mimic (follows FL)",     2, 0)
    app.addRadioButton("kinematic", "1", 2, 1)

    app.addLabel("l_egypt", "Egyptian",  3, 0)    
    app.addRadioButton("kinematic", "2", 3, 1)

    app.addLabel("l_kinmode", "Kinematic (default)", 4, 0)    
    app.addRadioButton("kinematic", "3",             4, 1)

    # foot coordinates mode (4) is missing as it is not implemented

    app.addLabel("l_joint", "Joint Angles", 6, 0)            
    app.addRadioButton("kinematic", "5",    6, 1)
    
    # set default mode
    app.setRadioButton("kinematic", "3", callFunction=False)
    # set to call function immediately on change
    app.setRadioButtonChangeFunction("kinematic", guiSetKinematicMode)

    app.addHorizontalSeparator()
    
    # ------------- Leg Compliance ---------------------------------------------
    
    # section title      
    app.addLabel("t_compliance", "Leg Compliance")
    app.addMessage("complianceMessage", """CAUTION: when turning off compliance the leg will snap back into rest position forcefully.""")
    # checkboxes to allow per-leg compliance
    app.addNamedCheckBox("Front Left", "complianceFL")
    app.addNamedCheckBox("Front Right", "complianceFR")
    app.addNamedCheckBox("Back Left", "complianceBL")
    app.addNamedCheckBox("Back Right", "complianceBR")
    # set them all to call the same function, every time a box is changed.
    app.setCheckBoxChangeFunction("complianceFL", guiSetLegCompliance)
    app.setCheckBoxChangeFunction("complianceFR", guiSetLegCompliance)
    app.setCheckBoxChangeFunction("complianceBL", guiSetLegCompliance)
    app.setCheckBoxChangeFunction("complianceBR", guiSetLegCompliance)
        
    app.addHorizontalSeparator()
    
    # ---------------- Battery stats ---------------------------------------------
    
    # section title    
    app.addLabel("t_battery", "Battery status")
    # battery data display
    app.addLabel("l_batt_mv", "batt mV: ......")
    app.addLabel("l_batt_ma", "batt mA: ......")

    app.addHorizontalSeparator()

    # ---------------  Head colour -----------------------------------------------
    
    # section title        
    app.addLabel("l_headColour", "Head Colour:")      
    # add sliders for RGB values
    app.addLabelScale("Red")
    app.setScaleRange("Red", 0, 255, curr=None)
    app.setScaleIncrement("Red", 1)
    app.showScaleValue("Red", show=True)    

    app.addLabelScale("Green")    
    app.setScaleRange("Green", 0, 255, curr=None)
    app.setScaleIncrement("Green", 1)
    app.showScaleValue("Green", show=True)    
    
    app.addLabelScale("Blue")    
    app.setScaleRange("Blue", 0, 255, curr=None)
    app.setScaleIncrement("Blue", 1)
    app.showScaleValue("Blue", show=True)    

    # add button to call head colour service
    app.addButton("Set Head Colour", guiSetHeadColour)

    app.stopFrame()

    # new frame for Joints and joint angle control =========================
    
    app.startFrame("CUSTOM CONTROL", row=0, column=1)
    
    # ----------- Joint angle data   -------------------------------------------------
    
    # section title
    app.addLabel("t_jointAngles", "Joint Angle Data from Robot:", 0, 0, 3)          
    # add box to enter number for report interval
    app.addNumericEntry("jointReportRate", 1, 0, 2)
    # set a suggested value for the entry, milliseconds
    app.setEntryDefault("jointReportRate", 100)
    # add button to call joint setup service
    app.addButton("Set Joint Report Rate", guiSetJointReportRate, 1, 2)
    
    # data display for the joint angles coming back from robot.
    # args labelname, labeltext, table row, col (optional)
    # (table args: row, col, colspan, rowspan)
    app.addLabel("l_joints_FLK", "FLK: ...", 2, 0) # row 2, col 0
    app.addLabel("l_joints_FLT", "FLT: ...", 3, 0) 
    app.addLabel("l_joints_FLH", "FLH: ...", 4, 0)

    app.addVerticalSeparator(2, 1, 1, 3) # separator takes up col 1, spans 3 rows

    app.addLabel("l_joints_FRK", "FRK: ...", 2, 2) # row 2, col 2
    app.addLabel("l_joints_FRT", "FRT: ...", 3, 2) 
    app.addLabel("l_joints_FRH", "FRH: ...", 4, 2)

    app.addHorizontalSeparator(5, 0, 3) # separator spans 3 cols

    app.addLabel("l_joints_BLK", "BLK: ...", 6, 0)    
    app.addLabel("l_joints_BLT", "BLT: ...", 7, 0) 
    app.addLabel("l_joints_BLH", "BLH: ...", 8, 0)

    app.addVerticalSeparator(6, 1, 1, 3)
    
    app.addLabel("l_joints_BRK", "BRK: ...", 6, 2)    
    app.addLabel("l_joints_BRT", "BRT: ...", 7, 2) 
    app.addLabel("l_joints_BRH", "BRH: ...", 8, 2)

    app.addHorizontalSeparator(9, 0, 3)

    # ---------- Joint Angle Controls --------------------------------------
    app.setSticky("")       
    app.addLabel("l_angleControls", "Joint Angle Controls (FK)", 10, 0, 3)

    # knee: min 400 (extended) max 800 (folded)
    # thigh: min 200 (down) max 600 (up)
    # hip: min 400 (CW from above) max 800 (CCW)
    
    app.addLabelScale("FLK", 11, 0) 
    app.setScaleRange("FLK", 400, 800, curr = 600) 
    app.showScaleValue("FLK", show=True)
    app.setScaleIncrement("FLK", 4) # This sets clicking in the blank area to nudge value by min useful value.  
    # This is set to 4 because the robot scales the value from 8bit in HID comms 
    # to 10bit internally; 1/4 the resolution unfortunately.
    
    # for immediate updates set a function to call for every change in the slider position     
    app.setScaleChangeFunction("FLK", guiJointControl)
    # inefficient to read 12 controls every time but probably better than 12 funcs

    app.addLabelScale("FLT", 12, 0)
    app.setScaleRange("FLT", 200, 600, curr=400) 
    app.showScaleValue("FLT", show=True)
    app.setScaleIncrement("FLT", 4)         
    app.setScaleChangeFunction("FLT", guiJointControl)

    app.addLabelScale("FLH", 13, 0)
    app.setScaleRange("FLH", 400, 800, curr=600)
    app.showScaleValue("FLH", show=True)
    app.setScaleIncrement("FLH", 4)        
    app.setScaleChangeFunction("FLH", guiJointControl)

    # --

    app.addLabelScale("FRK", 11, 2) 
    app.setScaleRange("FRK", 400, 800, curr = 600) 
    app.showScaleValue("FRK", show=True)
    app.setScaleIncrement("FRK", 4)                              
    app.setScaleChangeFunction("FRK", guiJointControl) 

    app.addLabelScale("FRT", 12, 2)
    app.setScaleRange("FRT", 200, 600, curr=400) 
    app.showScaleValue("FRT", show=True)
    app.setScaleIncrement("FRT", 4)                              
    app.setScaleChangeFunction("FRT", guiJointControl)

    app.addLabelScale("FRH", 13, 2)
    app.setScaleRange("FRH", 400, 800, curr=600) 
    app.showScaleValue("FRH", show=True)
    app.setScaleIncrement("FRH", 4)                              
    app.setScaleChangeFunction("FRH", guiJointControl)     

    # --
    app.setSticky("ew")       
    app.addHorizontalSeparator(14, 0, 3)
    app.setSticky("")       
    # --
                      
    app.addLabelScale("BLK", 15, 0) 
    app.setScaleRange("BLK", 400, 800, curr = 600) 
    app.showScaleValue("BLK", show=True)
    app.setScaleIncrement("BLK", 4)                              
    app.setScaleChangeFunction("BLK", guiJointControl) 

    app.addLabelScale("BLT", 16, 0)
    app.setScaleRange("BLT", 200, 600, curr=400) 
    app.showScaleValue("BLT", show=True)
    app.setScaleIncrement("BLT", 4)                              
    app.setScaleChangeFunction("BLT", guiJointControl)

    app.addLabelScale("BLH", 17, 0)
    app.setScaleRange("BLH", 400, 800, curr=600) 
    app.showScaleValue("BLH", show=True)
    app.setScaleIncrement("BLH", 4)                              
    app.setScaleChangeFunction("BLH", guiJointControl) 

    # --

    app.addLabelScale("BRK", 15, 2) 
    app.setScaleRange("BRK", 400, 800, curr = 600) 
    app.showScaleValue("BRK", show=True)
    app.setScaleIncrement("BRK", 4)                              
    app.setScaleChangeFunction("BRK", guiJointControl) 

    app.addLabelScale("BRT", 16, 2)
    app.setScaleRange("BRT", 200, 600, curr=400) 
    app.showScaleValue("BRT", show=True)
    app.setScaleIncrement("BRT", 4)                              
    app.setScaleChangeFunction("BRT", guiJointControl)

    app.addLabelScale("BRH", 17, 2)
    app.setScaleRange("BRH", 400, 800, curr=600) 
    app.showScaleValue("BRH", show=True)
    app.setScaleIncrement("BRH", 4)                              
    app.setScaleChangeFunction("BRH", guiJointControl)


    app.addHorizontalSeparator(18, 0, 3)

    # --------- Compass heading -----------------------------

    # section title
    app.addLabel("t_heading", "Compass Heading from Robot (relative):", 19, 0, 3)          
    # add box to enter number for report interval
    app.addNumericEntry("headingReportRate", 20, 0, 2)
    # set a suggested value for the entry, milliseconds
    app.setEntryDefault("headingReportRate", 100)
    # add button to call heading setup service
    app.addButton("Set Heading Report Rate", guiSetHeadingReportRate, 20, 2)
    
    # data display for the heading coming back from robot.
    app.addLabel("l_heading", "Deg: ...", 21, 0)     

    #----
    
    app.stopFrame()
    
    # new frame for Gait & Transform controls  =============================================
    
    app.startFrame("AUTO WALK CONTROL", row=0, column=2)


    # section title        
    app.addLabel("l_titleGaitParams", "Gait Parameters", 0, 0, 2)
    
    # add slider and button for each gait param
    app.setSticky("w") # align the label west   
    app.addLabel("l_StanceAngle", "Stance Angle degrees", 1, 0, 2)
    app.setSticky("ew") # allow the slider to expand east-west   
    app.addScale("StanceAngle", 2, 0)
    app.setScaleRange("StanceAngle", -45, 45, curr=0)
    app.setScaleIncrement("StanceAngle", 1) # this sets clicking in the blank area to nudge value by 1
    app.showScaleValue("StanceAngle", show=True) # this shows the selected value
    app.showScaleIntervals("StanceAngle", 45) # this shows the limits
    app.setSticky("") # stop the button expanding
    app.addNamedButton("Set", "b_StanceAngle", guiSetGaitParamStanceAngle, 2, 1)

    app.setSticky("w")       
    app.addLabel("l_StanceDistance", "Stance Distance mm", 3, 0, 2)
    app.setSticky("ew")       
    app.addScale("StanceDistance", 4, 0)
    app.setScaleRange("StanceDistance", 40, 200, curr=110)
    app.setScaleIncrement("StanceDistance", 1)
    app.showScaleValue("StanceDistance", show=True)
    app.showScaleIntervals("StanceDistance", 80)
    app.setSticky("")
    app.addNamedButton("Set", "b_StanceDistance", guiSetGaitParamStanceDistance, 4, 1)

    app.setSticky("w")       
    app.addLabel("l_WalkingSpeed", "Walking Speed deg/sec", 5, 0, 2)
    app.setSticky("ew")       
    app.addScale("WalkingSpeed", 6, 0)
    app.setScaleRange("WalkingSpeed", 0, 600, curr=328) # true value here
    app.setScaleIncrement("WalkingSpeed", 1)
    app.showScaleValue("WalkingSpeed", show=True)
    app.showScaleIntervals("WalkingSpeed", 300)
    app.setSticky("")
    app.addNamedButton("Set", "b_WalkingSpeed", guiSetGaitParamWalkingSpeed, 6, 1)

    app.setSticky("w")       
    app.addLabel("l_StepDuration", "Step Duration degrees", 7, 0, 2)
    app.setSticky("ew")       
    app.addScale("StepDuration", 8, 0)
    app.setScaleRange("StepDuration", 15, 90, curr=56)
    app.setScaleIncrement("StepDuration", 1)
    app.showScaleValue("StepDuration", show=True)
    app.showScaleIntervals("StepDuration", 25)
    app.setSticky("")
    app.addNamedButton("Set", "b_StepDuration", guiSetGaitParamStepDuration, 8, 1)

    app.setSticky("w")       
    app.addLabel("l_StepShift", "Step Shift degrees", 9, 0, 2)
    app.setSticky("ew")      
    app.addScale("StepShift", 10, 0)
    app.setScaleRange("StepShift", 15, 90, curr=71)
    app.setScaleIncrement("StepShift", 1)
    app.showScaleValue("StepShift", show=True)
    app.showScaleIntervals("StepShift", 25)
    app.setSticky("")
    app.addNamedButton("Set", "b_StepShift", guiSetGaitParamStepShift, 10, 1)    

    app.setSticky("w")       
    app.addLabel("l_StepHeight", "Step Height mm", 11, 0, 2)
    app.setSticky("ew")    
    app.addScale("StepHeight", 12, 0)
    app.setScaleRange("StepHeight", 0, 120, curr=90)
    app.setScaleIncrement("StepHeight", 1)
    app.showScaleValue("StepHeight", show=True)
    app.showScaleIntervals("StepHeight", 60)
    app.setSticky("")
    app.addNamedButton("Set", "b_StepHeight", guiSetGaitParamStepHeight, 12, 1)  

    app.setSticky("w")       
    app.addLabel("l_BodyHeight", "Body Height mm", 13, 0, 2)
    app.setSticky("ew")       
    app.addScale("BodyHeight", 14, 0)
    app.setScaleRange("BodyHeight", 0, 120, curr=36) # allowing it to be 0 here purely to make scale look nice
    app.setScaleIncrement("BodyHeight", 1)
    app.showScaleValue("BodyHeight", show=True)
    app.showScaleIntervals("BodyHeight", 60)
    app.setSticky("")
    app.addNamedButton("Set", "b_BodyHeight", guiSetGaitParamBodyHeight, 14, 1) 

    # use radio button for Steps as it only has 2 valid values
    app.setSticky("w")       
    app.addLabel("l_Steps", "Steps", 15, 0, 2)    
    app.addRadioButton("r_steps", "Trot", 16, 0)
    app.addRadioButton("r_steps", "Crawl", 16, 1)
    # set default mode
    app.setRadioButton("r_steps", "Trot", callFunction=False)
    # set to call function immediately on change
    app.setRadioButtonChangeFunction("r_steps", guiSetGaitParamSteps)

    app.setSticky("")       
    app.addLabel("l_Transform", "Transform", 17, 0, 2)
    
    app.setSticky("ns")       
    app.addLabelScale("Forward", 18, 0, 1, 2) # span both rows of the other 2 sliders
    app.setScaleVertical("Forward") # set this one vertical
    app.setScaleRange("Forward", 80, -80, curr=0) # appJar scales don't seem to do float
    app.showScaleValue("Forward", show=True)
    # for immediate updates set a function to call for every change in the slider position 
    app.setScaleChangeFunction("Forward", guiTransform) 

    app.setSticky("ew")       
    app.addLabelScale("Strafe", 18, 1)
    app.setScaleRange("Strafe", -80, 80, curr=0) 
    app.showScaleValue("Strafe", show=True)
    # for immediate updates set a function to call for every change in the slider position 
    app.setScaleChangeFunction("Strafe", guiTransform)

    app.setSticky("ew")       
    app.addLabelScale("Turn", 19, 1)
    app.setScaleRange("Turn", -80, 80, curr=0) 
    app.showScaleValue("Turn", show=True)
    # for immediate updates set a function to call for every change in the slider position 
    app.setScaleChangeFunction("Turn", guiTransform) 

    # add a button to reset Transform
    app.setSticky("")       
    app.addButton("Stop", guiTransformStop, 20, 0, 2) 

    app.stopFrame()

    # =========================================================

    # start this ROS node
    rospy.init_node('demo_client')
    
    # set up ROS subscribers(topic name, message type, callback function)
    rospy.Subscriber('battery_reports', Battery, batteryCallback)
    rospy.Subscriber('joints_return', JointAngles, jointsCallback)
    rospy.Subscriber('heading', UInt16, headingCallback)
    
    # (do this after preparing GUI else sub will try to use GUI funcs that aren't ready yet)

    # Set up ROS publishers(topic name, message type, queue size)
    jointsPub = rospy.Publisher('joints_send', JointAngles, queue_size=10)

    twistPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # --------------------------------------------------------------------

    # Start the ROS publisher loop in it's own thread.
    # Tell it what func to call when the ROS thread exits
    # and pass it references to the publisher objects
    app.threadCallback(ROSLoop, guiShutdown, jointsPub, twistPub)
    
    # start the GUI. Any code after this (in this func) will be blocked until GUI closes.
    app.go()

    print ("demo_client_node exiting")
    
    
###############################################################################

# startup

# create appJar library GUI object to manually control node
app = gui() #global

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

