
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    if (robot.links_geom_imported){
        // q_start_config = [
        //     -robot.origin.xyz[1],
        //     robot.origin.xyz[2],
        //     robot.origin.xyz[0],
        //     robot.origin.rpy[1],
        //     robot.origin.rpy[2],
        //     robot.origin.rpy[0]
        // ];
        q_start_config = [
            robot.origin.xyz[0],
            robot.origin.xyz[1],
            robot.origin.xyz[2],
            robot.origin.rpy[0],
            robot.origin.rpy[1],
            robot.origin.rpy[2]
        ];
    }else {
        q_start_config = [
            robot.origin.xyz[0],
            robot.origin.xyz[1],
            robot.origin.xyz[2],
            robot.origin.rpy[0],
            robot.origin.rpy[1],
            robot.origin.rpy[2]
        ];
    }


    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}


function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
    var q_rand = randomConfig();
    if (extendRRT(T_a,q_rand) != "Trapped"){
        if(connectRRT(T_b,q_new) == "Reached"){
            console.log("succeeded");
            kineval.motion_plan = Path(T_a,T_b);
            return "reached";
        }
    }
    // swap tree
    var temp = T_a;
    T_a = T_b;
    T_b = temp;

    return "extended";

    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    if (robot.links_geom_imported){
        temp_mesh.position.x = -vertex.vertex[1];
        temp_mesh.position.y = vertex.vertex[2];
        temp_mesh.position.z = vertex.vertex[0];
    }else {

        temp_mesh.position.x = vertex.vertex[0];
        temp_mesh.position.y = vertex.vertex[1];
        temp_mesh.position.z = vertex.vertex[2];
    }


    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function randomConfig(){
    var x_min = robot_boundary[0][0];
    var x_max = robot_boundary[1][0];
    var z_min = robot_boundary[0][2];
    var z_max = robot_boundary[1][2];

    if (robot.links_geom_imported){
        var rand_q = [
            Math.random() * (z_max-z_min) + z_min,
            -(Math.random() * (x_max-x_min) + x_min),
            0,
            robot.origin.rpy[0],
            robot.origin.rpy[1],
            Math.random() * (2*Math.PI) - Math.PI,
        ];
    }else {
        var rand_q = [
            Math.random() * (x_max-x_min) + x_min,
            0,
            Math.random() * (z_max-z_min) + z_min,
            robot.origin.rpy[0],
            Math.random() * (2*Math.PI) - Math.PI,
            robot.origin.rpy[2]
        ];
    }


    for (x in robot.joints) {
        rand_q = rand_q.concat(Math.random() * (2*Math.PI) - Math.PI);
        //rand_q = rand_q.concat(0);
    }

    var is_collided = kineval.poseIsCollision(rand_q);

    while(is_collided != false){
        
    if (robot.links_geom_imported){
            rand_q = [
            Math.random() * (z_max-z_min) + z_min,
            Math.random() * (x_max-x_min) + x_min,
            0,
            robot.origin.rpy[0],
            robot.origin.rpy[1],
            Math.random() * (2*Math.PI) - Math.PI,
        ];
    }else {
            rand_q = [
            Math.random() * (x_max-x_min) + x_min,
            0,
            Math.random() * (z_max-z_min) + z_min,
            robot.origin.rpy[0],
            Math.random() * (2*Math.PI) - Math.PI,
            robot.origin.rpy[2]
        ];
    }
        for (x in robot.joints) {
            rand_q = rand_q.concat(Math.random() * (2*Math.PI) - Math.PI);
            //rand_q = rand_q.concat(0);
        }
        is_collided = kineval.poseIsCollision(rand_q);
    }
    
    return rand_q;
}

function extendRRT(tree,q_target){

    var q_near_index = findNearestNeighbor(q_target,tree);
    q_new = newConfig(tree,q_target,q_near_index);
    //console.log("q_new",q_new);
    if (kineval.poseIsCollision(q_new) == false) {
        tree_add_vertex(tree,q_new);
        tree_add_edge(tree,q_near_index,tree.newest);

        if (is_q1_reached_q2(q_new,q_target)){
            return "Reached";
        } else {
            return "Advanced";
        }
    }
    return "Trapped"
}

function is_q1_reached_q2(q_1,q_2){        
    // min distance as reached
    var test_reached = 0;
    var step = 1;
    var step_ang = 0.5;

    if (robot.links_geom_imported){
        var dx = Math.abs(q_1[1] - q_2[1])<step;
        var dy = Math.abs(q_1[0] - q_2[0])<step;  
        var dpitch = Math.abs((q_1[5]-q_2[5])%Math.PI)<step_ang;
    }else {
        var dx = Math.abs(q_1[0] - q_2[0])<step;
        var dy = Math.abs(q_1[2] - q_2[2])<step;  
        var dpitch = Math.abs((q_1[4]-q_2[4])%Math.PI)<step_ang;
    }

    test_reached = dx && dy && dpitch;
    var d_joint = [];
    d_joint = d_joint.concat(test_reached);
    for (var i=6;i<q_1.length;i++){
        //d_joint = d_joint.concat(Math.abs(q_1[i] - q_2[i])<step_ang);
        d_joint = d_joint.concat(1);
    }

    return d_joint.every(c=>c==1);
}

// reutrn index of NearestNeighbor point
function findNearestNeighbor(q_target,tree){
    var x_min = robot_boundary[0][0];
    var x_max = robot_boundary[1][0];
    var z_min = robot_boundary[0][2];
    var z_max = robot_boundary[1][2];
    var num_joints = Object.keys(robot.joints).length;
    var dist_min = Math.sqrt((x_max-x_min)**2+(z_max-z_min)**2 + ((num_joints+1)*Math.PI)**2);
    var idex_min = 0;

    for (var i = 0;i<=tree.newest;i++){
        var q_i = tree.vertices[i].vertex;
        var dist_i = dist_q(q_target,q_i);

        if (dist_i<=dist_min){
            dist_min = dist_i;
            idex_min = i;
        }
    }
    return idex_min;
}

function dist_q(q_1,q_2){
    if (robot.links_geom_imported){
        var d_q = (q_1[0]-q_2[0])**2+(q_1[1]-q_2[1])**2;
        d_q = d_q + ((q_1[5]-q_2[5])%Math.PI)**2;
    }else {
        var d_q = (q_1[0]-q_2[0])**2+(q_1[2]-q_2[2])**2;
        d_q = d_q + ((q_1[4]-q_2[4])%Math.PI)**2;
    }

    // for (var i=6;i<q_1.length;i++){
    //     d_q = d_q + ((q_1[i]-q_2[i])%Math.PI)**2;
    // }

    return Math.sqrt(d_q);
}

function newConfig(tree,q_target,q_near_index){
    var step = 1;
    var q_near = tree.vertices[q_near_index].vertex;
    var step_ang = 0.25;

    if (robot.links_geom_imported){
        var x_near = q_near[1];
        var z_near = q_near[0];
        var angle = Math.atan2(q_target[0]-z_near,q_target[1]-x_near);
        var x_new = x_near + Math.cos(angle)*step;
        var z_new = z_near + Math.sin(angle)*step;
        
        var pitch_new = q_near[5] + (q_target[5]-q_near[5])*step_ang;
    
        var q_out = [z_new,x_new,q_near[2],q_near[3],q_near[4],pitch_new];
    }else {
        var x_near = q_near[0];
        var z_near = q_near[2];
        var angle = Math.atan2(q_target[2]-z_near,q_target[0]-x_near);
        var x_new = x_near + Math.cos(angle)*step;
        var z_new = z_near + Math.sin(angle)*step;
        var pitch_new = q_near[4] + (q_target[4]-q_near[4])*step_ang;
    
        var q_out = [x_new,q_near[1],z_new,q_near[3],pitch_new,q_near[5]];
    }

 
    var i = 6;
    for (x in robot.joints) {

        q_out = q_out.concat(q_near[i] + (q_target[i]-q_near[i])*step_ang);
        //q_out = q_out.concat(q_near[i]);
        i++;
    }

    return q_out;
}

function connectRRT(tree,q_target){
    //draw_2D_configuration(q_target, "queued");
    var S = extendRRT(tree,q_target);
    //console.log(S);
    while(S == "Advanced"){
        S = extendRRT(tree,q_target);
    } 
    return S;
}

// find Path for RRT connect
function Path(tree_a, tree_b){
    if (robot.links_geom_imported){
        var x_a0 = tree_a.vertices[0].vertex[0];
        var z_a0 = tree_a.vertices[0].vertex[1];
        var path_ab = [];
        var path_b = trace_from_end(tree_b);
        var path_a = trace_from_end(tree_a);
        // tree_a is start tree
        if (x_a0 == q_start_config[0] && z_a0 == q_start_config[1]){
            path_a = path_a.reverse();
            path_ab = path_a.concat(path_b);
        } else {
            path_b = path_b.reverse();
            path_ab = path_b.concat(path_a);
        }
    }else {
        var x_a0 = tree_a.vertices[0].vertex[0];
        var z_a0 = tree_a.vertices[0].vertex[2];
        var path_ab = [];
        var path_b = trace_from_end(tree_b);
        var path_a = trace_from_end(tree_a);
        // tree_a is start tree
        if (x_a0 == q_start_config[0] && z_a0 == q_start_config[2]){
            path_a = path_a.reverse();
            path_ab = path_a.concat(path_b);
        } else {
            path_b = path_b.reverse();
            path_ab = path_b.concat(path_a);
        }
    }



    return path_ab;
}

// trace tree from end to start
function trace_from_end(tree){
    var new_vertex = tree.vertices[tree.newest];
    var path = [new_vertex];
    var parent_vertex = new_vertex.edges[0];
    path.push(parent_vertex);
    parent_vertex.geom.material.color = {r:1,g:0,b:0};
    while(1){
        parent_vertex = parent_vertex.edges[0];
        if (robot.links_geom_imported){
            if((parent_vertex.vertex[0]==q_start_config[0] && parent_vertex.vertex[1]==q_start_config[1])||
            (parent_vertex.vertex[0]==q_goal_config[0] && parent_vertex.vertex[1]==q_goal_config[1])){
                path.push(parent_vertex);
                parent_vertex.geom.material.color = {r:1,g:0,b:0};
                break;
            } else {
                path.push(parent_vertex);
                parent_vertex.geom.material.color = {r:1,g:0,b:0};
            }
        }else {
            if((parent_vertex.vertex[0]==q_start_config[0] && parent_vertex.vertex[2]==q_start_config[2])||
            (parent_vertex.vertex[0]==q_goal_config[0] && parent_vertex.vertex[2]==q_goal_config[2])){
                path.push(parent_vertex);
                parent_vertex.geom.material.color = {r:1,g:0,b:0};
                break;
            } else {
                path.push(parent_vertex);
                parent_vertex.geom.material.color = {r:1,g:0,b:0};
            }
        }


    }
    return path;
}





