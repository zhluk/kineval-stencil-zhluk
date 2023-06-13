/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

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

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            var dx = Math.abs(q_init[0] - xpos);
            var dy = Math.abs(q_init[1] - ypos);
            //if (q_init[0] <= xpos + round && q_init[0] > xpos - round && q_init[1] <= ypos + round && q_init[1] > ypos - round){
            if (dx <= (eps)/2 && dy <= (eps)/2){
                G[iind][jind].queued = true;
                G[iind][jind].distance = 0;
                minheaper.insert(visit_queue,G[iind][jind]);
            }

        }
    }
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
 
     // call iteration for the selected search algorithm
     switch (search_alg) {
        case "depth-first":
            return DFS()
        case "breadth-first":
            return BFS()
        case "greedy-best-first":
            return GBFS()
        case "A-star":
            return A_star()
        default:
            console.warn('search_canvas: search algorithm not found, using A_star as default');
            return A_star()
    }
    
}

// A-star shortest path algorithm refered from graph_search lecture slide page 99
function A_star(){

    // while (visit_queue != empty) && current_node != goal
    if (visit_queue.length != 0){

        // cur_node ← dequeue(visit_queue, f_score)
        search_visited++;
        cur_node = minheaper.extract(visit_queue);
        cur_node.visited = true;
        draw_2D_configuration([cur_node.x,cur_node.y],"visited");

        //var is_cur_node_goal = q_goal[0] <= (cur_node.x + round) && q_goal[0] > (cur_node.x - round) && q_goal[1] <= (cur_node.y + round) && q_goal[1] > (cur_node.y - round)
        var dx = Math.abs(q_goal[0] - cur_node.x);
        var dy = Math.abs(q_goal[1] - cur_node.y);

        var is_cur_node_goal = dx <= (eps)/2 && dy <= (eps)/2;

        if(!is_cur_node_goal){

            var left_node = G[cur_node.i-1][cur_node.j];
            var right_node = G[cur_node.i+1][cur_node.j];
            var up_node = G[cur_node.i][cur_node.j-1];
            var down_node = G[cur_node.i][cur_node.j+1];
            var nbr_node = [];

            if (!testCollision([left_node.x,left_node.y]) && !left_node.visited)
                nbr_node.push(left_node);

            if (!testCollision([right_node.x,right_node.y]) && !right_node.visited)
                nbr_node.push(right_node);

            if (!testCollision([up_node.x,up_node.y]) && !up_node.visited)
                nbr_node.push(up_node);

            if (!testCollision([down_node.x,down_node.y]) && !down_node.visited)
                nbr_node.push(down_node);
            
            // for each nbr in not_visited(adjacent(cur_node))
            for (i=0;i<nbr_node.length;i++){
                // if dist_nbr > distcur_node + distance(nbr,cur_node)
                if (!nbr_node[i].queued && nbr_node[i].distance > (cur_node.distance + eps)){
                    // parent_nbr ← current_node
                    nbr_node[i].parent = cur_node;
                    // dist_nbr ← dist_cur_node + distance(nbr,cur_node)
                    nbr_node[i].distance = cur_node.distance + eps;
                    var dx = q_goal[0] - nbr_node[i].x;
                    var dy = q_goal[1] - nbr_node[i].y;
                    var h_score = Math.sqrt( dx*dx + dy*dy );
                    // f_score ← distance_nbr + line_distance_nbr,goal
                    nbr_node[i].priority = nbr_node[i].distance + h_score;
                    // enqueue(nbr to visit_queue)
                    minheaper.insert(visit_queue,nbr_node[i]);
                    nbr_node[i].queued = true
                    draw_2D_configuration([nbr_node[i].x,nbr_node[i].y],"queued");
                }
            }
            return "iterating"
        } else {
            search_iterate = false;
            drawHighlightedPathGraph(cur_node)
            return "succeeded"
        }
    } else {
        return "failed"
    }
}

// Depth-first search refered from graph_search lecture slide page 73
function DFS(){
    
    // while (visit_queue != empty) && current_node != goal
    if (visit_queue.length != 0){

        // cur_node ← dequeue(visit_queue, f_score)
        search_visited++;
        cur_node = visit_queue.pop();
        cur_node.visited = true;
        draw_2D_configuration([cur_node.x,cur_node.y],"visited");

        //var is_cur_node_goal = q_goal[0] < cur_node.x && q_goal[0] > cur_node.x - eps && q_goal[1] < cur_node.y && q_goal[1] > cur_node.y - eps

        var dx = Math.abs(q_goal[0] - cur_node.x);
        var dy = Math.abs(q_goal[1] - cur_node.y);

        var is_cur_node_goal = dx <= (eps)/2 && dy <= (eps)/2;

        if(!is_cur_node_goal){

            var left_node = G[cur_node.i-1][cur_node.j];
            var right_node = G[cur_node.i+1][cur_node.j];
            var up_node = G[cur_node.i][cur_node.j-1];
            var down_node = G[cur_node.i][cur_node.j+1];
            var nbr_node = [];

            if (!testCollision([right_node.x,right_node.y]) && !right_node.visited)
                nbr_node.push(right_node);

            if (!testCollision([left_node.x,left_node.y]) && !left_node.visited)
                nbr_node.push(left_node);

            if (!testCollision([down_node.x,down_node.y]) && !down_node.visited)
                nbr_node.push(down_node);

            if (!testCollision([up_node.x,up_node.y]) && !up_node.visited)
                nbr_node.push(up_node);

            // for each nbr in not_visited(adjacent(cur_node))
            for (i=0;i<nbr_node.length;i++){

                // if dist_nbr > distcur_node + distance(nbr,cur_node)
                
                if (nbr_node[i].distance > (cur_node.distance + eps)) {
                    // enqueue(nbr to visit_queue)
                    visit_queue.push(nbr_node[i]);
                    nbr_node[i].queued = true
                    draw_2D_configuration([nbr_node[i].x,nbr_node[i].y],"queued");
                    // parent_nbr ← current_node
                    nbr_node[i].parent = cur_node;
                    // dist_nbr ← dist_cur_node + distance(nbr,cur_node)
                    nbr_node[i].distance = cur_node.distance + eps;
                }
            }
            return "iterating"
        } else {
            search_iterate = false;
            drawHighlightedPathGraph(cur_node)
            return "succeeded"
        }
    } else {
        return "failed"
    }
}

// Breadth-first search refered from graph_search lecture slide page 78
function BFS(){
    
    // while (visit_queue != empty) && current_node != goal
    if (visit_queue.length != 0){

        // cur_node ← dequeue(visit_queue, f_score)
        search_visited++;
        cur_node = visit_queue.shift();
        cur_node.visited = true;
        draw_2D_configuration([cur_node.x,cur_node.y],"visited");

        //var is_cur_node_goal = q_goal[0] < cur_node.x && q_goal[0] > cur_node.x - eps && q_goal[1] < cur_node.y && q_goal[1] > cur_node.y - eps

        var dx = Math.abs(q_goal[0] - cur_node.x);
        var dy = Math.abs(q_goal[1] - cur_node.y);

        var is_cur_node_goal = dx <= (eps)/2 && dy <= (eps)/2;

        if(!is_cur_node_goal){

            var left_node = G[cur_node.i-1][cur_node.j];
            var right_node = G[cur_node.i+1][cur_node.j];
            var up_node = G[cur_node.i][cur_node.j-1];
            var down_node = G[cur_node.i][cur_node.j+1];
            var nbr_node = [];

            if (!testCollision([right_node.x,right_node.y]) && !right_node.visited)
                nbr_node.push(right_node);

            if (!testCollision([left_node.x,left_node.y]) && !left_node.visited)
                nbr_node.push(left_node);

            if (!testCollision([down_node.x,down_node.y]) && !down_node.visited)
                nbr_node.push(down_node);

            if (!testCollision([up_node.x,up_node.y]) && !up_node.visited)
                nbr_node.push(up_node);

            // for each nbr in not_visited(adjacent(cur_node))
            for (i=0;i<nbr_node.length;i++){

                // if dist_nbr > distcur_node + distance(nbr,cur_node)
                
                if (nbr_node[i].distance > (cur_node.distance + eps)) {
                    // enqueue(nbr to visit_queue)
                    visit_queue.push(nbr_node[i]);
                    nbr_node[i].queued = true
                    draw_2D_configuration([nbr_node[i].x,nbr_node[i].y],"queued");
                    // parent_nbr ← current_node
                    nbr_node[i].parent = cur_node;
                    // dist_nbr ← dist_cur_node + distance(nbr,cur_node)
                    nbr_node[i].distance = cur_node.distance + eps;

                }
            }
            return "iterating"
        } else {
            search_iterate = false;
            drawHighlightedPathGraph(cur_node)
            return "succeeded"
        }
    } else {
        return "failed"
    }
}

// Greedy Best-first search refered from graph_search lecture slide page 100 w/o g_score
function GBFS(){
    
    // while (visit_queue != empty) && current_node != goal
    if (visit_queue.length != 0){

        // cur_node ← dequeue(visit_queue, f_score)
        search_visited++;
        cur_node = minheaper.extract(visit_queue);
        cur_node.visited = true;
        draw_2D_configuration([cur_node.x,cur_node.y],"visited");

        //var is_cur_node_goal = q_goal[0] < cur_node.x && q_goal[0] > cur_node.x - eps && q_goal[1] < cur_node.y && q_goal[1] > cur_node.y - eps

        var dx = Math.abs(q_goal[0] - cur_node.x);
        var dy = Math.abs(q_goal[1] - cur_node.y);

        var is_cur_node_goal = dx <= (eps)/2 && dy <= (eps)/2;

        if(!is_cur_node_goal){

            var left_node = G[cur_node.i-1][cur_node.j];
            var right_node = G[cur_node.i+1][cur_node.j];
            var up_node = G[cur_node.i][cur_node.j-1];
            var down_node = G[cur_node.i][cur_node.j+1];
            var nbr_node = [];

            if (!testCollision([left_node.x,left_node.y]) && !left_node.visited)
                nbr_node.push(left_node);

            if (!testCollision([right_node.x,right_node.y]) && !right_node.visited)
                nbr_node.push(right_node);

            if (!testCollision([up_node.x,up_node.y]) && !up_node.visited)
                nbr_node.push(up_node);

            if (!testCollision([down_node.x,down_node.y]) && !down_node.visited)
                nbr_node.push(down_node);
            
            // for each nbr in not_visited(adjacent(cur_node))
            for (i=0;i<nbr_node.length;i++){
                // if dist_nbr > distcur_node + distance(nbr,cur_node)
                if (!nbr_node[i].queued && nbr_node[i].distance > (cur_node.distance + eps)){
                    // parent_nbr ← current_node
                    nbr_node[i].parent = cur_node;
                    // dist_nbr ← dist_cur_node + distance(nbr,cur_node)
                    nbr_node[i].distance = cur_node.distance + eps;
                    var dx = q_goal[0] - nbr_node[i].x;
                    var dy = q_goal[1] - nbr_node[i].y;
                    var h_score = Math.sqrt( dx*dx + dy*dy );
                    // f_score ← distance_nbr + line_distance_nbr,goal
                    nbr_node[i].priority =  h_score;
                    // enqueue(nbr to visit_queue)
                    minheaper.insert(visit_queue,nbr_node[i]);
                    nbr_node[i].queued = true
                    draw_2D_configuration([nbr_node[i].x,nbr_node[i].y],"queued");
                }
            }
            return "iterating"
        } else {
            search_iterate = false;
            drawHighlightedPathGraph(cur_node)
            return "succeeded"
        }
    } else {
        return "failed"
    }
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    var elntIdex = heap.length; // index of new element
    var prntIdex = Math.floor((elntIdex - 1) / 2);  // index of new element's parent

    heap.push(new_element); // add new_element to the heap array

    // Heap condition is true if new element added as root or if new element is less than or equal to its parent element
    var heaped = (elntIdex == 0) || (heap[prntIdex].priority <= heap[elntIdex].priority);

    while(!heaped)  {
        // Swap new element with its parent
        var temp = heap[prntIdex];
        heap[prntIdex] = heap[elntIdex];
        heap[elntIdex] = temp;

        // Update element and parent index
        elntIdex = prntIdex;
        prntIdex = Math.floor((elntIdex - 1) / 2);

        // Re-evaluate heap condition
        heaped = (elntIdex == 0) || (heap[prntIdex].priority <= heap[elntIdex].priority);
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var N = heap.length; // length of the heap
    var root  = heap[0]; // root of heap

    // Swap root and last element
    heap[0] = heap[N-1];
    heap[N-1] = root;

    // return root when there are only two elements
    if (N <= 2){
        heap.pop();
        return root
    } else {
        
        // pop the last element which is the min
        heap.pop(); 

        // initalize element index to root
        var elntIdex = 0
        var l = 2 * elntIdex + 1; // left child
        var r = 2 * elntIdex + 2; // right child
        N = heap.length;
        // Heap condition is ture if root is the only element or root is smaller than its children
        var heaped = (l >= N) || (r >= N && heap[elntIdex].priority <= heap[l].priority) || (heap[elntIdex].priority <= heap[l].priority && heap[elntIdex].priority <= heap[r].priority);

        while(!heaped) {
            // console.log("swap happened");
            var smallest = elntIdex;  // initialize smallest index

            // Swap root with its smallest child
            if (l < N && heap[l].priority < heap[elntIdex].priority)
                smallest = l;

            if (r < N && heap[r].priority < heap[smallest].priority)
                smallest = r; 
                
            if (smallest != elntIdex) {
                var temp = heap[elntIdex];
                heap[elntIdex] = heap[smallest];
                heap[smallest] = temp;
            }

            // Update element and parent index
            elntIdex = smallest;
            l = 2 * elntIdex + 1; // left child
            r = 2 * elntIdex + 2; // right child

            // Re-evaluate heap condition
            heaped = (l >= N) || (r >= N && heap[elntIdex].priority <= heap[l].priority) || (heap[elntIdex].priority <= heap[l].priority && heap[elntIdex].priority <= heap[r].priority);
        }

        // return original root
        return root
    }
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;