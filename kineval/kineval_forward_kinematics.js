
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

    kineval.buildFKTransforms();



    // STENCIL: call kineval.buildFKTransforms();
}

    // STENCIL: implement buildFKTransforms, which kicks off
    //   a recursive traversal over links and 
    //   joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // To use the keyboard interface, assign the global variables 
    //   "robot_heading" and "robot_lateral", 
    //   which represent the z-axis (heading) and x-axis (lateral) 
    //   of the robot's base in its own reference frame, 
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form 
    //   as 4x1 homogenous matrices

    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

    kineval.buildFKTransforms = function buildFKTransforms(){
        mstack = [generate_identity()];
        var cur_node = robot.base;
        traverseFKBase(mstack,cur_node);
        
    }


function traverseFKBase(mstack,cur_node){
  

    if (robot.links_geom_imported){

        var R_w_b = rotation_matrix(robot.origin.rpy);
        var tx = robot.origin.xyz[0];
        var ty = -robot.origin.xyz[1];
        var tz = robot.origin.xyz[2];
        var D_w_b = generate_translation_matrix(tx,ty,tz);

        var R_convert = [
            [0,1,0,0],
            [0,0,1,0],
            [1,0,0,0],
            [0,0,0,1]];

        var R_convertback = [
            [0,0,1,0],
            [1,0,0,0],
            [0,1,0,0],
            [0,0,0,1]];
        
        // robot_heading = [[robot_heading_f[2]],[robot_heading_f[0]],[robot_heading_f[1]]];
        // robot_lateral = [[robot_lateral_f[2]],[robot_lateral_f[0]],[robot_lateral_f[1]]];

        var mstack_top = matrix_multiply(mstack[mstack.length-1],matrix_multiply(R_convert,matrix_multiply(D_w_b,R_w_b)));
       
        mstack.push(mstack_top);
    
        robot.links[robot.base].xform = mstack[mstack.length-1];

        robot_heading = matrix_multiply(matrix_multiply(D_w_b,R_w_b),[[1],[0],[0],[1]]);
        robot_lateral = matrix_multiply(matrix_multiply(D_w_b,R_w_b),[[0],[-1],[0],[1]]);


    }else{

        var R_w_b = rotation_matrix(robot.origin.rpy);
        var tx = robot.origin.xyz[0];
        var ty = robot.origin.xyz[1];
        var tz = robot.origin.xyz[2];
        var D_w_b = generate_translation_matrix(tx,ty,tz);

        var mstack_top = matrix_multiply(mstack[mstack.length-1],matrix_multiply(D_w_b,R_w_b));
        mstack.push(mstack_top);

        robot.links[robot.base].xform = mstack[mstack.length-1];

        robot_heading = matrix_multiply(robot.links[robot.base].xform,[[0],[0],[1],[1]]);
        robot_lateral = matrix_multiply(robot.links[robot.base].xform,[[1],[0],[0],[1]]);
    }

    // console.log(cur_node);
    // console.log(robot.links[robot.base].xform);
    // console.log(mstack_top);
    traverseFKJoint(mstack,cur_node);
    
}

function traverseFKJoint(mstack,cur_node){
    var child_joints = robot.links[cur_node].children;
    
    for (x in child_joints) {
        cur_node = child_joints[x];

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

        // get quaternion for joint rotation
        if (robot.joints[cur_node].type ==='revolute' || robot.joints[cur_node].type ==='continuous' || typeof robot.joints[cur_node].type === 'undefined'){
            var q = kineval.quaternionFromAxisAngle(robot.joints[cur_node].axis,robot.joints[cur_node].angle);
            R_u = kineval.quaternionToRotationMatrix(q);
        }

        if (robot.joints[cur_node].type ==='prismatic'){
            var pris_ang = [0,0,0];
            pris_ang[0] = robot.joints[cur_node].axis[0] * robot.joints[cur_node].angle;
            pris_ang[1] = robot.joints[cur_node].axis[1] * robot.joints[cur_node].angle;
            pris_ang[2] = robot.joints[cur_node].axis[2] * robot.joints[cur_node].angle;
    
            //console.log(pris_ang);
            R_u = generate_translation_matrix(pris_ang[0],pris_ang[1],pris_ang[2]);
        }

        var mstack_top = matrix_multiply(mstack[mstack.length-1],matrix_multiply(D_l_j,matrix_multiply(R_l_j,R_u)));
        mstack.push(mstack_top);

        robot.joints[cur_node].xform = mstack[mstack.length-1];
        
        // console.log(cur_node);
        // console.log(robot.joints[cur_node].xform);
        // console.log(mstack_top);
        traverseFKLink(mstack,cur_node);
    }

}

function traverseFKLink(mstack,cur_node){
    var child_link = robot.joints[cur_node].child;

    robot.links[child_link].xform = mstack[mstack.length-1];

    if (typeof robot.links[child_link].children === 'undefined'){
        mstack.pop();
    } else{
        cur_node = child_link;
        // console.log(cur_node);
        // console.log(robot.links[child_link].xform);
        // console.log(mstack);
        traverseFKJoint(mstack,cur_node);
        mstack.pop();
    }
}

function rotation_matrix(rpy){
    var angle_x = rpy[0];
    var angle_y = rpy[1];
    var angle_z = rpy[2];

    var R_x = generate_rotation_matrix_X(angle_x);
    var R_y = generate_rotation_matrix_Y(angle_y);
    var R_z = generate_rotation_matrix_Z(angle_z);


    var R = matrix_multiply(R_z,matrix_multiply(R_y,R_x));

    return R;
}
