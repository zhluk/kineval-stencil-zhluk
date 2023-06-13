//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};

    q.a = Math.cos(angle/2);
    q.b = axis[0]*Math.sin(angle/2);
    q.c = axis[1]*Math.sin(angle/2);
    q.d = axis[2]*Math.sin(angle/2);

    return q;
}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};

    norm_q1 = Math.sqrt(q1.a**2+q1.b**2+q1.c**2+q1.d**2);

    q.a = q1.a / norm_q1;
    q.b = q1.b / norm_q1;
    q.c = q1.c / norm_q1;
    q.d = q1.d / norm_q1;

    return q;
}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    
    q.a = q1.a*q2.a - q1.b*q2.b - q1.c*q2.c - q1.d*q2.d;
    q.b = q1.a*q2.b + q1.b*q2.a + q1.c*q2.d - q1.d*q2.c;
    q.c = q1.a*q2.c - q1.b*q2.d + q1.c*q2.a + q1.d*q2.b;
    q.d = q1.a*q2.d + q1.b*q2.c - q1.c*q2.b + q1.d*q2.a;

    return q;
}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
    // returns 4-by-4 2D rotation matrix

    var M_rot = [
        [q.a**2+q.b**2-q.c**2-q.d**2,2*(q.b*q.c-q.a*q.d),2*(q.a*q.c+q.b*q.d),0],
        [2*(q.b*q.c+q.a*q.d),q.a**2-q.b**2+q.c**2-q.d**2,2*(q.c*q.d-q.a*q.b),0],
        [2*(q.b*q.d-q.a*q.c),2*(q.a*q.b+q.c*q.d),q.a**2-q.b**2-q.c**2+q.d**2,0],
        [0,0,0,1]
    ];

    return M_rot;

}