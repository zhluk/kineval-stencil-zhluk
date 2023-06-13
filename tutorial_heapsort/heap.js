/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    var elntIdex = heap.length; // index of new element
    var prntIdex = Math.floor((elntIdex - 1) / 2);  // index of new element's parent

    heap.push(new_element); // add new_element to the heap array

    // Heap condition is true if new element added as root or if new element is less than or equal to its parent element
    var heaped = (elntIdex == 0) || (heap[prntIdex] <= heap[elntIdex]);

    while(!heaped)  {
        // Swap new element with its parent
        var temp = heap[prntIdex];
        heap[prntIdex] = heap[elntIdex];
        heap[elntIdex] = temp;

        // Update element and parent index
        elntIdex = prntIdex;
        prntIdex = Math.floor((elntIdex - 1) / 2);

        // Re-evaluate heap condition
        heaped = (elntIdex == 0) || (heap[prntIdex] <= heap[elntIdex]);
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
        // console.log("less than 2 left")
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
        var heaped = (l >= N) || (r >= N && heap[elntIdex] <= heap[l]) || (heap[elntIdex] <= heap[l] && heap[elntIdex] <= heap[r]);

        while(!heaped) {
            // console.log("swap happened");
            var smallest = elntIdex;  // initialize smallest index

            // Swap root with its smallest child
            if (l < N && heap[l] < heap[elntIdex])
                smallest = l;

            if (r < N && heap[r] < heap[smallest])
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
            heaped = (l >= N) || (r >= N && heap[elntIdex] <= heap[l]) || (heap[elntIdex] <= heap[l] && heap[elntIdex] <= heap[r]);
        }

        // return original root
        return root
    }
}


// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






