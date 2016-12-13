
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



/* STENCIL START */ 
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle)
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // initialize tree from start configuration
    q_init = q_start_config;
    T_a = tree_init(q_init);

    // initialize tree from goal configuration
    q_goal = q_goal_config;
    T_b = tree_init(q_goal);

    // keep track of which tree is connected to the start configuration
    a_start_tree = true;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    verbose_console_rrt = false;
    if (verbose_console_rrt)
        console.log("planner initialized");
}



function robot_rrt_planner_iterate() {
    var i,exret,conret;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED) 2: rrt_star

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();
        eps = 0.5;
        var q_rand = randomConfig(T_a.vertices[0].vertex.length);

        if (rrt_alg === 2) {
            var q_nearest = findNearestNeighbor(q_rand,T_a);
            var ret = extendRRT(q_rand,T_a);
            if(ret === 'trapped'){
                return 'failed';
            }
            var q_new = T_a.vertices[T_a.newest];
            var radius = 10*eps;
            var q_min = chooseParent(T_a.vertices[q_nearest.index],q_new,T_a,radius);
            rewire(q_min,q_new,T_a,radius);
            var q_g = findNearestNeighbor(q_goal,T_a);
            var d = q_dist(q_new.vertex,q_goal);
            if(d < eps){
                var path = RRTstarpath(T_a.vertices[q_g.index]);
                find_path(path);
                return 'reached'; 
            }
            else{
                return 'extended';
            }
        }
        else {            
            exret = extendRRT(q_rand,T_a);
            if ((exret !=="trapped")) {
                conret = connectRRT(T_b,T_a.vertices[T_a.newest].vertex);
                if (conret==="reached") {
                    rrt_iterate = false;
                    path_a = path_dfs(T_a,T_a.vertices[0],T_a.vertices[T_a.newest]);
                    path_b = path_dfs(T_b,T_b.vertices[0],T_b.vertices[T_b.newest]);
                    find_path(path_a);
                    find_path(path_b);
                    kineval.motion_plan = [];
                    var robot_path_idx = 0;
                    kineval.motion_plan_traversal_index = 0;
                    var p1 = a_start_tree ? path_a : path_b;
                    var p2 = a_start_tree ? path_b : path_a;
                    for (i=p1.length-1;i>=0;i--) {
                        kineval.motion_plan[robot_path_idx] = p1[i];
                        robot_path_idx += 1;
                    }
                    for (i=0;i<p2.length;i++) {
                        kineval.motion_plan[robot_path_idx] = p2[i];
                        robot_path_idx += 1;
                    }
                    return conret;
                }
            }
            var temp = T_a;
            T_a = T_b;
            T_b = temp;
            a_start_tree = !a_start_tree;
            rrt_iter_count++; 
        }
        normalize_joint_state(T_a);
        normalize_joint_state(T_b);
    }
    return false;
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
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
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
function normalize_joint_state(tree){
    for (i=0;i<tree.vertices.length;i++) {
        for (j=3;j<tree.vertices[i].length;j++) {
            // normalize joint state and enforce joint limits for non-base
            if (j > 5) {
                if (typeof robot.joints[q_index[j]].type !== 'undefined')
                    if ((robot.joints[q_index[j]].type === 'prismatic')||(robot.joints[q_index[j]].type === 'revolute'))
                        tree.vertices[i].vertex[j] = normalize_joint(T_a.vertices[i].vertex[j],robot.joints[q_index[j]].limit.lower,robot.joints[q_index[j]].limit.upper);
                    else 
                        tree.vertices[i].vertex[j] = normalize_angle(T_a.vertices[i].vertex[j]);
                else 
                        tree.vertices[i].vertex[j] = normalize_angle(T_a.vertices[i].vertex[j]);
            }
            else
                tree.vertices[i].vertex[j] = normalize_angle(T_a.vertices[i].vertex[j]);
        }
    }
}
function normalize_angle(angle){
    while(angle > 2*Math.PI){
        angle -= 2*Math.PI;
    }
    while(angle < 0){
        angle += 2*Math.PI;
    }
    return angle
}
function normalize_joint(value,lower,upper) {
    return Math.min(Math.max(value,lower),upper);
}

function extendRRT(q,tree){
    var eps = 0.5;
    var q_nearest = findNearestNeighbor(q,tree);
    var idx_q_nearest = q_nearest.index;
    var q_near_config = q_nearest.vertex;
    var newconfig = newConfig(q,q_nearest.vertex,eps);
    if(!kineval.poseIsCollision(newconfig)){
        var q_new = newconfig;
        tree_add_vertex(tree,q_new);
        tree_add_edge(tree,tree.vertices.length-1,idx_q_nearest);
        if (q_dist(q_new,q) < eps){
            return "reached";
        }
        else{
            return "advanced"
        }
    }
    return "trapped";
}

function connectRRT(tree,q){
    var ret;
    do{
        ret = extendRRT(q,tree);
    }
    while(ret === 'advanced')
    return ret;
}
function randomConfig(length){
    q_rand = [];
    var i;
    for (i=0;i<length;i++) {
        if (i < 6){
            q_rand.push(0);
        }
        else if (typeof robot.joints[q_index[i]].type !== 'undefined')
            if ((robot.joints[q_index[i]].type === 'prismatic')||(robot.joints[q_index[i]].type === 'revolute'))
                q_rand.push(Math.random()*((robot.joints[q_index[i]].limit.upper)-(robot.joints[q_index[i]].limit.lower))+(robot.joints[q_index[i]].limit.lower));
            else
                q_rand.push(Math.random()*2*Math.PI);
        else
            q_rand.push(Math.random()*2*Math.PI);

    }
    q_rand[0] = (robot_boundary[1][0]-robot_boundary[0][0])*Math.random()+robot_boundary[0][0];
    q_rand[2] = (robot_boundary[1][2]-robot_boundary[0][2])*Math.random()+robot_boundary[0][2];
    q_rand[4] = Math.random()*2*Math.PI;
    return q_rand;
}

function newConfig(q_end,q_start,eps){
    var sumofsquare = 0;
    var q = [];
    var q_unit = [];
    
    for(var i = 0;i<q_end.length;i++){
        q[i] = q_end[i]-q_start[i];
        sumofsquare += q[i]*q[i];
    }
    var mag = Math.sqrt(sumofsquare);
    for(var i = 0;i<q_end.length;i++){
        q_unit[i] = q[i] / mag *eps + q_start[i];
    }
    return q_unit;
}

function findNearestNeighbor(q,tree){
    dist_min = q_dist(q,tree.vertices[0].vertex);
    idx_min = 0;
    for(var i=1;i<tree.vertices.length;i++){
        var d = q_dist(q,tree.vertices[i].vertex);
        if(d <dist_min){
            dist_min = d;
            idx_min = i;
        }
    }
    var ret={};
    ret.index = idx_min;
    ret.vertex = tree.vertices[idx_min].vertex;
    return ret;
}
function q_dist(q1,q2){
    var dist = 0;
    for(var i = 0;i<q1.length;i++){
        dist+= (q1[i] - q2[i]) * (q1[i] - q2[i]);
    }
    return Math.sqrt(dist);
}
function find_path(path) {
    for (i=0;i<path.length;i++) {
        path[i].geom.material.color = {r:1,g:0,b:0};
    }

}
function path_dfs(tree,q_start ,q_end){
    if(q_start === q_end){
        return [q_start];
    }
    q_start.visited = true;
    var path;
    for(var i = 0;i<q_start.edges.length;i++){
        if(!q_start.edges[i].visited){
            path = path_dfs(tree,q_start.edges[i],q_end);
        }
        if(path){
            return path.concat(q_start)
        }
    }
    return false;
}
function chooseParent(q_nearest,q_new,tree,radius){
    var q_min = q_nearest;
    var cost_min = q_dist(q_min.vertex,q_new.vertex)+q_min.cost; 
    for(var i = 1;i<tree.vertices.length-1;i++){
        var node = tree.vertices[i];
        var cost_node = node.cost+q_dist(node.vertex,q_new.vertex);
        if(!checkIntersect(q_new.vertex,node.vertex) 
        && q_dist(node.vertex,q_new.vertex) < radius 
        &&  cost_node < cost_min
        && node !== q_new){
            q_min = node;
            cost_min = cost_node; 
        }
    }
    q_new.cost = q_min.cost + q_dist(q_min.vertex,q_new.vertex);
    q_new.parent = q_min;
    return q_min;
}
function rewire(q_min,q_new,tree,radius){
    for(var i = 1;i<tree.vertices.length-1;i++){
        var node = tree.vertices[i];
        var cost = q_new.cost+q_dist(node.vertex,q_new.vertex);
        if(!checkIntersect(q_new.vertex,node.vertex) 
        && node !== q_new.parent
        && node !== q_new
        && q_dist(node.vertex,q_new.vertex) < radius 
        && cost < node.cost){
            node.parent = q_new;
            node.cost = cost;
            tree.vertices[i] = node;
        }
    }
}
function checkIntersect(q1,q2){
    var q_new = q1;
    while(q_dist(q_new,q2) > eps){
        if(kineval.poseIsCollision(q_new)){
            return true;
        }
        q_new = newConfig(q2,q_new,eps);
    }
    return false;
}
function RRTstarpath(q){
    var path = [];
    var q_t = q;
    while(q_t !== null){
        path.push(q_t);
        q_t = q_t.parent;
    }
    return path;
}







