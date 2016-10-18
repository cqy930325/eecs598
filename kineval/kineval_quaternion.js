//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,theta){
	var q = {};
	q.a = Math.cos(theta/2);
	q.b = axix[0] * Math.sin(theta/2);
	q.c = axix[1] * Math.sin(theta/2);
	q.d = axix[2] * Math.sin(theta/2);
	return q;
}

kineval.quaternionNormalize = function quaternion_normalize(q){
	var norm = 0;
	norm = Math.sqrt(q.a*q.a+q.b*q.b+q.c*q.c+q.d*q.d);
	var norm_q = {}
	norm_q.a = q.a/norm;
	norm_q.b = q.b/norm;
	norm_q.c = q.c/norm;
	norm_q.d = q.d/norm;
	return norm_q;
}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix(q){
	mat = [
		    [q.a*q.a+q.b*q.b-q.c*q.c-q.d*q.d, 2*(q.b*q.c-q.a*q.d),             2*(q.a*q.c+q.b*q.d),0], 
			[2*(q.b*q.c+q.a*q.d),             q.a*q.a-q.b*q.b+q.c*q.c-q.d*q.d, 2*(q.c*q.d-q.a*q.b),0],
			[2*(q.b*q.d-q.a*q.c),             2*(q.a*q.b+q.c*q.d),             q.a*q.a-q.b*q.b-q.c*q.c+q.d*q.d,0],
			[0,0,0,1]
		];
	return mat;
}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2){
    var q = {};
    q.a = q1.a*a2.a - q1.b*a2.b - q1.c*a2.c - q1.d*a2.d;
    q.b = q1.a*a2.b + q1.b*a2.a + q1.c*a2.d - q1.d*a2.c; 
    q.c = q1.a*a2.c - q1.b*a2.d + q1.c*a2.a + q1.d*a2.b; 
    q.d = q1.a*a2.d + q1.b*a2.c - q1.c*a2.b + q1.d*a2.a; 
    return q;
}
