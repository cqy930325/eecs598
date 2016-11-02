
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 
    var isreached = true;
    var reach_thresh = 0.1;
    var x;
    for (x in robot.joints) {
        if (Math.abs(robot.joints[x].angle-kineval.params.setpoint_target[x]) > reach_thresh){   
            isreached = false;
        }
    }

    if (isreached) {
        kineval.params.dance_pose_index = kineval.params.dance_pose_index+1;
        if (kineval.params.dance_pose_index == kineval.params.dance_sequence_index.length)
            kineval.params.dance_pose_index = 0;
    }
    for (x in robot.joints) {
        var idx = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];
        var action = kineval.setpoints[idx];
        kineval.params.setpoint_target[x] = action[x];
    }
    // STENCIL: implement FSM to cycle through dance pose setpoints
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    var x;  
    for (x in robot.joints) {
        robot.joints[x].servo.p_desired = kineval.params.setpoint_target[x]; 
        robot.joints[x].control = robot.joints[x].servo.p_gain * (robot.joints[x].servo.p_desired - robot.joints[x].angle);
    }
}



