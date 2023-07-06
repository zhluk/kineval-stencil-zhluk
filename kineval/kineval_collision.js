
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles

        q_robot_config = [
            robot.origin.xyz[0],
            robot.origin.xyz[1],
            robot.origin.xyz[2],
            robot.origin.rpy[0],
            robot.origin.rpy[1],
            robot.origin.rpy[2]
        ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if (robot.links_geom_imported){
        if ((-q[1]<robot_boundary[0][0])||(-q[1]>robot_boundary[1][0])||(q[0]<robot_boundary[0][2])||(q[0]>robot_boundary[1][2]))
        return robot.base;
    }else {
        if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;
    }


    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    
    return robot_collision_forward_kinematics(q);

}

function robot_collision_forward_kinematics(q){

    var mstack = [generate_identity()];

    if (robot.links_geom_imported){
        var R_w_b = rotation_matrix([q[3],q[4],q[5]]);
        var tx = q[0];
        var ty = -q[1];
        var tz = q[2];
        var D_w_b = generate_translation_matrix(tx,ty,tz);

        var R_convert = [
            [0,1,0,0],
            [0,0,1,0],
            [1,0,0,0],
            [0,0,0,1]];

        var mstack_top = matrix_multiply(mstack[mstack.length-1],matrix_multiply(R_convert,matrix_multiply(D_w_b,R_w_b)));
       
        mstack.push(mstack_top);

    } else{
        var R_w_b = rotation_matrix([q[3],q[4],q[5]]);
        var tx = q[0];
        var ty = q[1];
        var tz = q[2];
        var D_w_b = generate_translation_matrix(tx,ty,tz);
        var mstack_top = matrix_multiply(mstack[mstack.length-1],matrix_multiply(D_w_b,R_w_b));
        mstack.push(mstack_top);
    }


    var collsion_link = traverse_collision_forward_kinematics_link(robot.links[robot.base],mstack_top,q);
    if (collsion_link!= false){
        return collsion_link;
    }

    return false;
}



function traverse_collision_forward_kinematics_link(link,mstack,q) {

    // test collision FK
    //console.log(link);

    if (typeof link.visual !== 'undefined') {
        var local_link_xform = matrix_multiply(mstack,generate_translation_matrix(link.visual.origin.xyz[0],link.visual.origin.xyz[1],link.visual.origin.xyz[2]));
    }
    else {
        var local_link_xform = matrix_multiply(mstack,generate_identity());
    }

    // test collision by transforming obstacles in world to link space
/*
    mstack_inv = matrix_invert_affine(mstack);
*/
    mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}

function traverse_collision_forward_kinematics_joint(joint,mstack,q){
    var cur_node = joint.name;
    var R_l_j = rotation_matrix(robot.joints[cur_node].origin.rpy);
    var tx = robot.joints[cur_node].origin.xyz[0];
    if (robot.links_geom_imported){
        var ty = robot.joints[cur_node].origin.xyz[1];
    } else {
        var ty = robot.joints[cur_node].origin.xyz[1];
    }

    var tz = robot.joints[cur_node].origin.xyz[2];
    var D_l_j = generate_translation_matrix(tx,ty,tz);

    var R_u = generate_identity();

    var joint_angle = q[q_names[cur_node]];
    // get quaternion for joint rotation
    if (robot.joints[cur_node].type ==='revolute' || robot.joints[cur_node].type ==='continuous' || typeof robot.joints[cur_node].type === 'undefined'){
        var q_ang = kineval.quaternionFromAxisAngle(robot.joints[cur_node].axis,joint_angle);
        R_u = kineval.quaternionToRotationMatrix(q_ang);
    }

    if (robot.joints[cur_node].type ==='prismatic'){
        var pris_ang = [0,0,0];
        pris_ang[0] = robot.joints[cur_node].axis[0] * joint_angle;
        pris_ang[1] = robot.joints[cur_node].axis[1] * joint_angle;
        pris_ang[2] = robot.joints[cur_node].axis[2] * joint_angle;

        //console.log(pris_ang);
        R_u = generate_translation_matrix(pris_ang[0],pris_ang[1],pris_ang[2]);
    }

    var mstack_top = matrix_multiply(mstack,matrix_multiply(D_l_j,matrix_multiply(R_l_j,R_u)));
    var child_link = robot.joints[cur_node].child;
    return traverse_collision_forward_kinematics_link(robot.links[child_link],mstack_top,q);
}

