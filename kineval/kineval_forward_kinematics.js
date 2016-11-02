
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

kineval.buildFKTransforms = function buildFKTransforms(){
    kineval.traverseFKBase();
    if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        kineval.traverseFKLink(robot.links[robot.base],matrix_multiply(robot.origin.xform,offset_xform));
    }else{
        kineval.traverseFKLink(robot.links[robot.base],robot.origin.xform);
    }
}

kineval.traverseFKLink = function traverseFKLink(link,xform){
    link.xform = matrix_multiply(xform,generate_identity());
    var i;
    if (typeof link.children !== 'undefined'){
        for(i = 0;i<link.children.length;i++){
            kineval.traverseFKJoint(robot.joints[link.children[i]],link.xform);
        }
    }else{
        return;
    }
}

kineval.traverseFKJoint = function traverseFKJoint(joint,xform){
    var joint_origin_xform = kineval.generate_transform_matrix(joint.origin.xyz,joint.origin.rpy);
        var new_xform = generate_identity();
    if(joint.type === "prismatic"){
        var trans_pris = []
        trans_pris[0] = joint.angle * joint.axis[0];
        trans_pris[1] = joint.angle * joint.axis[1];
        trans_pris[2] = joint.angle * joint.axis[2];
        new_xform = generate_translation_matrix(trans_pris);
    }
    else if((joint.type === 'revolute')||(joint.type === 'continuous')||(joint.type === undefined)){
        var q = kineval.quaternionFromAxisAngle(joint.axis,joint.angle);
        new_xform = kineval.quaternionToRotationMatrix(kineval.quaternionNormalize(q));
    }
    joint.xform = matrix_multiply(matrix_multiply(xform,joint_origin_xform),new_xform); 
    kineval.traverseFKLink(robot.links[joint.child],joint.xform);
}

kineval.traverseFKBase = function traverseFKBase(){
    robot.origin.xform = kineval.generate_transform_matrix(robot.origin.xyz,robot.origin.rpy);
    robot_heading = matrix_multiply(robot.origin.xform,[[0],[0],[1],[1]]);
    robot_lateral = matrix_multiply(robot.origin.xform,[[1],[0],[0],[1]]);
}

kineval.generate_transform_matrix = function generate_transform(xyz,rpy){

    var trans = generate_translation_matrix(xyz);
    var x_rot = generate_rotation_matrix_X(rpy[0]);
    var y_rot = generate_rotation_matrix_Y(rpy[1]);
    var z_rot = generate_rotation_matrix_Z(rpy[2]);
    return matrix_multiply(trans,matrix_multiply(z_rot,matrix_multiply(y_rot,x_rot)));
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

