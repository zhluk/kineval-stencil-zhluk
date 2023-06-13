/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
    var q_rand = randomConfig();
    //draw_2D_configuration(q_rand, "queued");
    rtn_extend = extendRRT(T_a,q_rand);
    // check if q_new is the goal
    if (is_q1_reached_q2(q_new,q_goal)){

        console.log("succeeded");
        var path = trace_from_end(T_a);
        search_iterate = false;
        drawHighlightedPath(path);
        return "succeeded";
    } else {
        return "extended";
    }

}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    var q_rand = randomConfig();
    //draw_2D_configuration(q_rand, "queued");
    if (extendRRT(T_a,q_rand) != "Trapped"){
        if(connectRRT(T_b,q_new) == "Reached"){
            //console.log("succeeded");
            var path = Path(T_a,T_b);
            search_iterate = false;
            drawHighlightedPath(path);
            return "succeeded";
        }
    }
    // swap tree
    var temp = T_a;
    T_a = T_b;
    T_b = temp;

    return "extended";

}

function iterateRRTStar() {

    var q_rand = randomConfig();
    //draw_2D_configuration(q_rand, "queued");
    var q_near_index = findNearestNeighbor(q_rand,T_a);
    q_new = newConfig(T_a,q_rand,q_near_index);
    if (!testCollision(q_new)) {
        // find a set of neighboring nodes
        var Q_near_index = neighbor_node(T_a,q_new);
        var q_min_index = choose_parent(T_a,Q_near_index,q_near_index,q_new);

        insertTreeVertex(T_a,q_new);
        insertTreeEdge(T_a,q_min_index,T_a.newest);

        rewire(T_a,Q_near_index,q_min_index,q_new);
    }

    if (is_q1_reached_q2(q_new,q_goal)){

        console.log("succeeded");
        var path = trace_from_end(T_a);
        search_iterate = false;
        drawHighlightedPath(path);
        return "succeeded";
    } else {
        return "extended";
    }

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

// For RRT*
function neighbor_node(tree,q){
    var Q_neighbor_index = [];
    var dist_near = 0.5;
    for (var i = 0;i<tree.newest;i++){
        var q_i = tree.vertices[i].vertex;
        var x_i = q_i[0];
        var y_i = q_i[1];
        var dist_i = Math.sqrt((q[0]-x_i)**2+(q[1]-y_i)**2);

        if (dist_i<=dist_near){
            Q_neighbor_index.push(i);
        }
    }
    return Q_neighbor_index;
}

function choose_parent(tree,Q_neighbor_index,z_nearest_index,z_new){
    var q_nearest = tree.vertices[z_nearest_index].vertex;
    var z_min = z_nearest_index;
    var c_min = cost(tree,z_nearest_index) + Math.sqrt((q_nearest[0]-z_new[0])**2+(q_nearest[1]-z_new[1])**2);

    for (var i=0;i<Q_neighbor_index.length;i++){
        var q_i = tree.vertices[Q_neighbor_index[i]].vertex;
        var test = false;
        var x_i = q_i;
        // check collison between q_new and q_i
        while(1){
            var angle = Math.atan2(z_new[1]- x_i[1],z_new[0]- x_i[0]);
            var x_new =  x_i[0] + Math.cos(angle)*eps;
            var y_new =  x_i[1] + Math.sin(angle)*eps;
            x_i = [x_new,y_new];

            test = testCollision(x_i);
            if(test||is_q1_reached_q2(x_i,z_new)){
                break;
            }
        }

        if(!test){
            var c_i = cost(tree,Q_neighbor_index[i]) + Math.sqrt((q_i[0]-z_new[0])**2+(q_i[1]-z_new[1])**2);
            if (c_i < c_min) {
                z_min = Q_neighbor_index[i];
                c_min = c_i;
            }
        }
    }
    return z_min
}

function cost(tree, q_index){
    var new_vertex = tree.vertices[q_index];
    var dist = 0;
    if(new_vertex.edges.length == 0){
        return dist;
    }
    parent_vertex = new_vertex.edges[0];
    dist = dist + Math.sqrt((parent_vertex.vertex[0]-new_vertex.vertex[0])**2+(parent_vertex.vertex[1]-new_vertex.vertex[1])**2);
    while(1){
        new_vertex = parent_vertex;
        parent_vertex = new_vertex.edges[0];
        if((parent_vertex.vertex[0]==q_init[0] && parent_vertex.vertex[1]==q_init[1])||
        (parent_vertex.vertex[0]==q_goal[0] && parent_vertex.vertex[1]==q_goal[1])){
            dist = dist + Math.sqrt((parent_vertex.vertex[0]-new_vertex.vertex[0])**2+(parent_vertex.vertex[1]-new_vertex.vertex[1])**2);
            break;
        } else {
            dist = dist + Math.sqrt((parent_vertex.vertex[0]-new_vertex.vertex[0])**2+(parent_vertex.vertex[1]-new_vertex.vertex[1])**2);
        }
    }
    return dist;
}

function rewire(tree,Q_neighbor_index,z_min,z_new){
    for (var i=0;i<Q_neighbor_index.length;i++){
        if (Q_neighbor_index[i] != z_min){
            var q_i = tree.vertices[Q_neighbor_index[i]].vertex;
            var test = false;
            var x_i = z_new;
            // check collison between z_new and q_i
            while(1){
                var angle = Math.atan2(q_i[1]- x_i[1],q_i[0]- x_i[0]);
                var x_new =  x_i[0] + Math.cos(angle)*eps;
                var y_new =  x_i[1] + Math.sin(angle)*eps;
                x_i = [x_new,y_new];

                test = testCollision(x_i);
                if(test||is_q1_reached_q2(x_i,q_i)){
                    break;
                }
            }

            if(!test){
                var c_i = cost(tree,tree.newest) + Math.sqrt((q_i[0]-z_new[0])**2+(q_i[1]-z_new[1])**2);
                if (c_i < cost(tree,Q_neighbor_index[i])) {
                    tree.vertices[Q_neighbor_index[i]].edges[0] = tree.vertices[tree.newest];
                }
            }
        }

    }
}


// For RRT and RRT_connect
function randomConfig(){
    
    var rand_x = Math.random() * 7 - 1.5;
    var rand_y = Math.random() * 7 - 1.5;
    var is_collided = testCollision([rand_x,rand_y]);

    while(is_collided){
        rand_x = Math.random() * 7 - 1.5;
        rand_y = Math.random() * 7 - 1.5;
        is_collided = testCollision([rand_x,rand_y]);
    }
    
    return [rand_x,rand_y];
}

function extendRRT(tree,q_target){

    var q_near_index = findNearestNeighbor(q_target,tree);
    q_new = newConfig(tree,q_target,q_near_index);
    //console.log("q_new",q_new);
    if (!testCollision(q_new)) {
        insertTreeVertex(tree,q_new);
        insertTreeEdge(tree,q_near_index,tree.newest);

        if (is_q1_reached_q2(q_new,q_target)){
            return "Reached";
        } else {
            return "Advanced";
        }
    }
    return "Trapped"
}

function is_q1_reached_q2(q1,q2){
    var dx = Math.abs(q1[0] - q2[0]);
    var dy = Math.abs(q1[1] - q2[1]);
    return dx <= eps && dy <= eps;
}

// reutrn index of NearestNeighbor point
function findNearestNeighbor(q_target,tree){
    var dist_min = Math.sqrt(2)*9;
    var idex_min = 0;
    for (var i = 0;i<tree.newest;i++){
        var q_i = tree.vertices[i].vertex;
        var x_i = q_i[0];
        var y_i = q_i[1];
        var dist_i = Math.sqrt((q_target[0]-x_i)**2+(q_target[1]-y_i)**2);

        if (dist_i<=dist_min){
            dist_min = dist_i;
            idex_min = i;
        }
    }
    return idex_min;
}

function newConfig(tree,q_target,q_near_index){
    var q_near = tree.vertices[q_near_index].vertex;
    var x_near = q_near[0];
    var y_near = q_near[1];
    var angle = Math.atan2(q_target[1]-y_near,q_target[0]-x_near);
    var x_new = x_near + Math.cos(angle)*eps;
    var y_new = y_near + Math.sin(angle)*eps;
    return [x_new,y_new];
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
    var x_a0 = tree_a.vertices[0].vertex[0];
    var y_a0 = tree_a.vertices[0].vertex[1];
    var path_ab = [];
    var path_b = trace_from_end(tree_b);
    var path_a = trace_from_end(tree_a);
    // tree_a is start tree
    if (x_a0 == q_init[0] && y_a0 == q_init[0]){
        path_a = path_a.reverse();
        path_ab = path_a.concat(path_b);
    } else {
        path_b = path_b.reverse();
        path_ab = path_b.concat(path_a);
    }
    return path_ab;
}

// trace tree from end to start
function trace_from_end(tree){
    var new_vertex = tree.vertices[tree.newest];
    var path = [new_vertex];
    var parent_vertex = new_vertex.edges[0];
    path.push(parent_vertex);
    while(1){
        parent_vertex = parent_vertex.edges[0];
        if((parent_vertex.vertex[0]==q_init[0] && parent_vertex.vertex[1]==q_init[1])||
        (parent_vertex.vertex[0]==q_goal[0] && parent_vertex.vertex[1]==q_goal[1])){
            path.push(parent_vertex);
            break;
        } else {
            path.push(parent_vertex);
        }
    }
    return path;
}