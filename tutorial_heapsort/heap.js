
// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element);
    bubble_up(heap,heap.length-1);
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
    var ans = heap[0];
    var end = heap.pop();
    if (heap.length > 0){
    	heap[0] = end;
    	sink_down(heap,0);
    }
    return ans;
}
minheaper.extract = minheap_extract;
function sink_down(heap,n){
	while(true){
		var ind_Rchild = (n+1)*2, ind_Lchild = ind_Rchild - 1;
		var tmp = null;
		if (ind_Lchild  < heap.length){
			var var_Lchild = heap[ind_Lchild];
			if (var_Lchild < heap[n])
				tmp = ind_Lchild;
		}
		if (ind_Rchild < heap.length){
			var var_Rchild = heap[ind_Rchild];
			if (tmp == null && var_Rchild < heap[n])
				tmp= ind_Rchild;
			if (tmp != null && var_Rchild < tmp)
				tmp = ind_Rchild;
		} 
		if (tmp != null){
			var value = heap[n];
			heap[n] = heap[tmp];
			heap[tmp] = value;
			n = tmp;
		}
		if (tmp == null){
			break;
		}	
	}
}
function bubble_up(heap,n){
	while(n > 0){
		var ind_parent = Math.floor((n+1)/2)-1;
		var val_parent = heap[ind_parent];
		if (heap[n] >= val_parent)
			break;
		heap[ind_parent] = heap[n];
		heap[n] = val_parent;
		n = ind_parent;
	}
}

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object






