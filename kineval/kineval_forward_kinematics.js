
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();

    kineval.buildFKTransforms();
}

kineval.buildFKTransforms() = function buildFKTransforms(){
    kineval.traverseFKBase();
    if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        kineval.traverseFKLink(robot.links[robot.base],matrix_multiply(robot.origin.xform,offset_xform));
    }else{
        kineval.traverseFKLink(robot.links[robot.base],robot.origin.xform);
    }
}

kineval.traverseFKLink() = function traversalFKLink(link,xform){
    link.xform = matrix_multiply(xform,generate_identity());
    var i;
    if (typeof link.children !== 'undefined'){
        for(i = 0;i<link.children.length,i++){
            kineval.traverseFKLink(robot.joints[link.children[i]],link.xform);
        }
    }else{
        return;
    }
}

kineval.traverseFKJoint() = function traverseFKJoint(joint,xform){
    var trans = generate_translation_matrix(joint.origin.xyz);
    var x_rot = generate_rotation_matrix_X(joint.origin.rpy[0]);
    var y_rot = generate_rotation_matrix_X(joint.origin.rpy[1]);
    var z_rot = generate_rotation_matrix_X(jointt.origin.rpy[2]);
    joint_origin_xform = matrix_multiply(trans,matrix_multiply(z_rot,matrix_multiply(y_rot,x_rot)));
    if(joint.type === "prismatic"){
        var trans_pris = []
        trans_pris[0] = joint.angle * joint.axis[0];
        trans_pris[1] = joint.angle * joint.axis[1];
        trans_pris[2] = joint.angle * joint.axis[2];
        joint_xform = generate_translation_matrix(trans_pris);
    }
    else if ((joint.type === 'revolute')||(joint.type === 'continuous'))
}

kineval.traverseFKBase() = function traverseFKBase(){
    var trans = generate_translation_matrix(robot.origin.xyz);
    var x_rot = generate_rotation_matrix_X(robot.origin.rpy[0]);
    var y_rot = generate_rotation_matrix_X(robot.origin.rpy[1]);
    var z_rot = generate_rotation_matrix_X(robot.origin.rpy[2]);
    robot.origin.xform = matrix_multiply(trans,matrix_multiply(z_rot,matrix_multiply(y_rot,x_rot)));
    robot_heading = matrix_multiply(robot.origin.xform,[[0],[0],[1],[1]]);
    robot_lateral = matrix_multiply(robot.origin.xform,[[1],[0],[0],[1]]);
}



    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

