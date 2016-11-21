
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    var joint  = endeffector_joint;
    var joints = [];
    while(robot.joints[joint].parent !== robot.base){
        joints.push(joint);
        joint = robot.links[robot.joints[joint].parent].parent;
    }
    joints.push(joint);
    var ef_world = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local); 
    var J = [[],[],[],[],[],[]];   
    var origin = [[0],[0],[0],[1]];
    var i = 0;
    for (i = 0;i < joints.length;i++){
        var chain_joint = robot.joints[joints[i]];
        var joint_world = matrix_multiply(chain_joint.xform,origin);
        var joint_axis =  [[chain_joint.axis[0]],[chain_joint.axis[1]],[chain_joint.axis[2]],[0]];
        var joint_axis_world =  matrix_multiply(chain_joint.xform,joint_axis);
        var jointtoend = [ef_world[0][0]-joint_world[0][0],ef_world[1][0]-joint_world[1][0],ef_world[2][0]-joint_world[2][0]];
        var z = [joint_axis_world[0][0],joint_axis_world[1][0],joint_axis_world[2][0]];
        if (chain_joint.type !== undefined && chain_joint.type === 'prismatic'){
            J[0][i] = z[0];
            J[1][i] = z[1];
            J[2][i] = z[2];
            J[3][i] = 0;
            J[4][i] = 0;
            J[5][i] = 0;
        }
        else{
            var zxjointend = vector_cross(z,jointtoend);
            J[0][i] = zxjointend[0];
            J[1][i] = zxjointend[1];
            J[2][i] = zxjointend[2];
            J[3][i] = z[0];
            J[4][i] = z[1];
            J[5][i] = z[2];
        }
    }
        //https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    var R = matrix_copy(chain_joint.xform);
    var sy = Math.sqrt(R[0][0] * R[0][0] +  R[1][0] * R[1][0]);
    if (sy > 0.000001){
        var x = Math.atan2(R[2][1],R[2][2]);
        var y = Math.atan2(R[2][0],sy);
        var z = Math.atan2(R[1][0],R[0][0]);
    }else{
        var x = Math.atan2(R[1][2],R[1][1]);
        var y = Math.atan2(R[2][0],sy);
        var z = 0;
    }
    if (kineval.params.ik_orientation_included){
        dx = [
            [ endeffector_target_world.position[0][0]-ef_world[0][0] ],
            [ endeffector_target_world.position[1][0]-ef_world[1][0] ],
            [ endeffector_target_world.position[2][0]-ef_world[2][0] ]
            ,[ endeffector_target_world.orientation[0]-x]
            ,[ endeffector_target_world.orientation[1]-y ]
            ,[ endeffector_target_world.orientation[2]-z ]
        ]; 
    }
    else{
        dx = [
            [ endeffector_target_world.position[0][0]-ef_world[0][0] ],
            [ endeffector_target_world.position[1][0]-ef_world[1][0] ],
            [ endeffector_target_world.position[2][0]-ef_world[2][0] ]
            ,[0]
            ,[0]
            ,[0]
        ]; 
    }
    var dq = matrix_multiply(matrix_pseudoinverse(J),dx);
    var ik_scale = kineval.params.ik_steplength;
    for (i=0;i<joints.length;i++) {
        robot.joints[joints[i]].control += ik_scale*dq[i];
    }
}



