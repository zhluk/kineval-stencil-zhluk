
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
    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
    kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
    kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
    kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
    kineval.params.trial_ik_random.targets += 1;
    textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }

}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    robot.dx = [];              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    robot.jacobian = [];        // Jacobian matrix of current IK iteration matrix size: 6 x N
    robot.dq = [];              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length

    // endeffector_target_world ʹ target pose of end effector for IK, has .position and .orientation
    // endeffector_joint ʹ string name of joint connected to end effector
    // endeffector_position_local ʹ position of end effector with respect to local frame

    var alpha = kineval.params.ik_steplength;       // size of step to take along configuration gradient when updating control
    var flag_IK = kineval.params.ik_pseudoinverse;  // Boolean flag denoting which method to use (Jacobian transpose vs pseudoinverse)

    var p_t_w = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local); // endeffector position in world frame
    // endeffector_target_world
    var x_d = [];
    x_d[0] = endeffector_target_world.position[0];
    x_d[1] = endeffector_target_world.position[1];
    x_d[2] = endeffector_target_world.position[2];
    x_d[3] = [endeffector_target_world.orientation[0]];
    x_d[4] = [endeffector_target_world.orientation[1]];
    x_d[5] = [endeffector_target_world.orientation[2]];

    // endeffector_position_world
    x_n = [];
    x_n[0] = p_t_w[0];
    x_n[1] = p_t_w[1];
    x_n[2] = p_t_w[2];
    if (kineval.params.ik_orientation_included){
        var R = [[robot.joints[endeffector_joint].xform[0][0], robot.joints[endeffector_joint].xform[0][1],robot.joints[endeffector_joint].xform[0][2]],
        [robot.joints[endeffector_joint].xform[1][0], robot.joints[endeffector_joint].xform[1][1],robot.joints[endeffector_joint].xform[1][2]],
        [robot.joints[endeffector_joint].xform[2][0], robot.joints[endeffector_joint].xform[2][1],robot.joints[endeffector_joint].xform[2][2]]];
        var eul = rotm2eul(R);
        x_n[3] = [eul[0]];
        x_n[4] = [eul[1]];
        x_n[5] = [eul[2]];
        //console.log(eul);
    }else{
        x_n[3] = [endeffector_target_world.orientation[0]];
        x_n[4] = [endeffector_target_world.orientation[1]];
        x_n[5] = [endeffector_target_world.orientation[2]];
    }

    //  desired endeffector displacement
    robot.dx = [[x_d[0] - x_n[0]],
                [x_d[1] - x_n[1]],
                [x_d[2] - x_n[2]],
                [x_d[3] - x_n[3]],
                [x_d[4] - x_n[4]],
                [x_d[5] - x_n[5]]];

    var cur_joint = endeffector_joint;
    joint_list = [];
    var flag_parent = 1;
    while(flag_parent){

        var o_i_w = matrix_multiply(robot.joints[cur_joint].xform,[[0],[0],[0],[1]]);        // joint origin in world frame
        var r = [p_t_w[0] - o_i_w[0],
                 p_t_w[1] - o_i_w[1],
                 p_t_w[2] - o_i_w[2]];      // vector from joint orgin to endeffector in world frame
        var joint_axis = [[robot.joints[cur_joint].axis[0]],
                          [robot.joints[cur_joint].axis[1]],
                          [robot.joints[cur_joint].axis[2]],
                          [1]];
        var k_i_w = matrix_multiply(robot.joints[cur_joint].xform,joint_axis);               // joint axis in world frame
        var k = [k_i_w[0] - o_i_w[0],
                 k_i_w[1] - o_i_w[1],
                 k_i_w[2] - o_i_w[2]];    // joint rotation axis in world frame

        var J_vi = vector_cross(k,r);
        var J_wi = k;

        if (typeof robot.joints[cur_joint].type === 'undefined' || robot.joints[cur_joint].type ==='revolute' || robot.joints[cur_joint].type ==='continuous'){
            robot.jacobian.unshift([J_vi[0], J_vi[1], J_vi[2],J_wi[0],J_wi[1],J_wi[2]]);
            joint_list.push(cur_joint);
        }

        if (robot.joints[cur_joint].type ==='prismatic'){
            robot.jacobian.unshift([k[0], k[1], k[2],0,0,0]);
            joint_list.push(cur_joint);
        }

        if(robot.joints[cur_joint].parent===robot.base){
            flag_parent = 0;
        } else {
            cur_joint = robot.links[robot.joints[cur_joint].parent].parent;
        }

    }

    robot.jacobian = matrix_transpose(robot.jacobian);


    if (flag_IK){
        robot.dq =  matrix_multiply(matrix_pseudoinverse(robot.jacobian),robot.dx);
    } else {
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian),robot.dx);
    }

    // update joint angles
    var i = robot.dq.length - 1;
    for (x in joint_list) {
        robot.joints[joint_list[x]].control = alpha*robot.dq[i][0];
        i--;
    }
}

function rotm2eul(R) {
    var x,y,z,eul;
    //xyz
    x = Math.atan2(-R[1][2] , R[2][2]);
    y = Math.asin(R[0][2]);
    z = Math.atan2(-R[0][1], R[0][0]);

    return eul=[x,y,z];
}


