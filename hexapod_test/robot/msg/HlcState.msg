std_msgs/Header header
# 0: kInit, 1: kSitDown, 2: kStandUp, 3: kHighLevelCmd
uint8 control_mode
# 0: kWaitHwReady, 1: kWaitMotorInit, 2: kNormalCtrl, 3:kStop
uint8 fsm_state
# 0: kNone, 1: kJoint, 2: kJointLcm, 3: kFoot, 4: kPlan
uint8 command_type
