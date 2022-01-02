/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "graphs.h"
#include <fstream>

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

struct cell{

    int parent;
    double g;

    cell()
        : parent(-1)
        , g(-1)
    {
    }
};

PRoadmap prm;

vector<vector<double>> saved_map;

double time_limit = 60.0;  // time limit in seconds

typedef pair<double, int> listPair;

double get_cost(vector<double*> planned_path, int numofDOFs)
{
    double cost = 0;
    for(int i = 0; i < planned_path.size() - 1; i++)
    { 
        cost += dist_joints(planned_path[i], planned_path[i+1], numofDOFs);
    }
    return cost;
}

vector<double*> getPathRRTConnect(RRTree ta, RRTree tb)
{
    int id = ta.get_numofvertices()-1;
    stack<double*> stk;
    vector<double*> Path;
    mexPrintf("path 1\n");
    // return Path;
    while(ta.grid[id] != 0)
    {
        stk.push(ta.vertices[id].joints);
        int temp_id = ta.grid[id];
        id = temp_id;
    }
    stk.push(ta.vertices[0].joints);

    while(!stk.empty())
    {
        Path.push_back(stk.top());
        stk.pop();
    }
    
    id = tb.grid[tb.get_numofvertices()-1];
 
    while(tb.grid[id] != 0)
    {
        Path.push_back(tb.vertices[id].joints);
        int temp_id = tb.grid[id];
        id = temp_id;
        // mexPrintf("path77 %d\n", count);
    }
    Path.push_back(tb.vertices[0].joints);    
    return Path;
}

vector<double*> getPathRRT(unordered_map<int, int>& grid, int vi, vector<vertex>& vertices)
{
    int id = vi;
    vector<double*> Path;
 
    while (!(grid[id] == id))
    {
        Path.push_back(vertices[id].joints);
        int temp_id = grid[id];
        id = temp_id;
    }
    Path.push_back(vertices[id].joints);
    return Path;
}

vector<double*> getPath(unordered_map<int, cell>& grid, int vi, vector<vertex>& vertices)
{
    int id = vi;
    vector<double*> Path;
 
    while (!(grid[id].parent == id))
    {
        Path.push_back(vertices[id].joints);
        int temp_id = grid[id].parent;
        id = temp_id;
    }
    Path.push_back(vertices[id].joints);
    return Path;
}

/////////////////////////////////////////////// PLANNER ///////////////////////////////////////////////////

static void RRTPlan(
        double*	map,
        int x_size,
        int y_size,
        double* armstart_anglesV_rad,
        double* armgoal_anglesV_rad,
        int numofDOFs,
        double*** plan,
        int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    int max_samples = 200000, num_samples=0;
    bool found_tree = false;
    vector<double*> planned_path;

    if(!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) || !IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size))
    {
        mexPrintf(":::invalid start or goal:::\n");
        return;
    }

    clock_t tStart = clock();

    RRTree rrt(numofDOFs);

    rrt.init(armstart_anglesV_rad, armgoal_anglesV_rad);

    // build the tree
    while(num_samples++ < max_samples)
    {
        // mexPrintf("sample %d\n", num_samples);
        double* rand_angle = random_sample(numofDOFs, armgoal_anglesV_rad, 0.9);
        unordered_set<int> neighbors = get_neighbors(rand_angle, rrt.vertices, 1, numofDOFs);
        double* qnear = rrt.vertices[*neighbors.begin()].joints;
        double* qnew = new_config(qnear, rand_angle, map, x_size, y_size, numofDOFs, 0.6);
        if(qnew!=NULL)
        {           
            vertex v = {rrt.get_numofvertices(), qnew};
            rrt.add_vertex(v);
            rrt.grow_tree(v.id, *neighbors.begin());
            if(goal_reached(qnew, armgoal_anglesV_rad, numofDOFs, 0.6) && connect(qnew, armgoal_anglesV_rad, map, x_size, y_size, numofDOFs))
            {
                found_tree = true;
                vertex v = {rrt.get_numofvertices(), armgoal_anglesV_rad};
                rrt.add_vertex(v);
                rrt.grow_tree(v.id, rrt.vertices.size()-2);
                mexPrintf("Time taken RRT: %.3fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
                break;
            }
            if((double)(clock() - tStart)/CLOCKS_PER_SEC>time_limit) break;
        }
    }
    if(found_tree)
    {
        planned_path = getPathRRT(rrt.grid, rrt.get_numofvertices()-1, rrt.vertices);
        *planlength = planned_path.size();
        *plan = (double**) malloc(*planlength*sizeof(double*));
        for(int i = 0; i < *planlength; i++)
        {
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
            for(int j = 0; j < numofDOFs; j++)
            {
                (*plan)[i][j] = planned_path[planned_path.size()-i-1][j];
            }
        }
        double plan_cost = get_cost(planned_path, numofDOFs);
        mexPrintf("path cost RRT: %.3f\n", plan_cost);
    }
    else
    {
        mexPrintf("ERROR: No path found!!!\n");
    }
    mexPrintf("num vertices RRT: %d\n", rrt.get_numofvertices());
    
    rrt.vertices.clear();
    rrt.grid.clear();
    return;
}

static void RRTConnectPlan(
        double*	map,
        int x_size,
        int y_size,
        double* armstart_anglesV_rad,
        double* armgoal_anglesV_rad,
        int numofDOFs,
        double*** plan,
        int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

    if(!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) || !IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size))
    {
        mexPrintf("invalid start or goal\n");
        return;
    }

    int max_samples = 100000, num_samples=0;
    bool found_tree = false;
    vector<double*> planned_path;

    clock_t tStart = clock();

    RRTree ta(numofDOFs);
    RRTree tb(numofDOFs);

    ta.init(armstart_anglesV_rad, armgoal_anglesV_rad);
    tb.init(armgoal_anglesV_rad, armstart_anglesV_rad);

    // build the tree
    while(num_samples++ < max_samples)
    {
        // mexPrintf("num_samples rrt-connect: %d\n", num_samples);
        double* rand_angle = random_sample(numofDOFs, armgoal_anglesV_rad, 1.1);
        unordered_set<int> neighbors = get_neighbors(rand_angle, ta.vertices, 1, numofDOFs);
        double* qnear = ta.vertices[*neighbors.begin()].joints;
        double* qnew = new_config(qnear, rand_angle, map, x_size, y_size, numofDOFs, 0.5);
        if(qnew!=NULL)
        {           
            vertex v1 = {ta.get_numofvertices(), qnew};
            ta.add_vertex(v1);
            ta.grow_tree(v1.id,*neighbors.begin());
            unordered_set<int> neighbors = get_neighbors(qnew, tb.vertices, 1, numofDOFs);
            double* qb = tb.vertices[*neighbors.begin()].joints;
            double* qbnew = new_config(qb, qnew, map,x_size,y_size, numofDOFs, 0.5);
            if(qbnew!=NULL)
            {
                vertex v2 = {tb.get_numofvertices(), qbnew};
                tb.add_vertex(v2);
                tb.grow_tree(v2.id,*neighbors.begin());
            }
            if(same_config(qbnew, qnew, numofDOFs))
            {
                found_tree = true; 
                mexPrintf("Time taken RRTConnect: %.3fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

                break;
            }            
        }
        swap(ta, tb);
    }

    if(same_config(ta.vertices[0].joints,armstart_anglesV_rad, numofDOFs))
    {
        swap(ta,tb);    // swap trees T.a and T.b
    }

    if(found_tree)
    {
        planned_path = getPathRRTConnect(ta,tb);
        *planlength = planned_path.size();
        *plan = (double**) malloc(*planlength*sizeof(double*));
        for(int i = 0; i < *planlength; i++)
        {
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
            for(int j = 0; j < numofDOFs; j++)
            {
                (*plan)[i][j] = planned_path[planned_path.size()-i-1][j];
            }
        }
        double plan_cost = get_cost(planned_path, numofDOFs);
        mexPrintf("path cost RRT-C: %.3f\n", plan_cost);
    }
    else
    {
        mexPrintf("ERROR: No path found!!!\n");
    }
    mexPrintf("num vertices RRTC: %d\n", ta.get_numofvertices()+tb.get_numofvertices());
    
    ta.vertices.clear();
    ta.grid.clear();
    tb.vertices.clear();
    tb.grid.clear();

    return;
}

static void RRTStarPlan(
        double*	map,
        int x_size,
        int y_size,
        double* armstart_anglesV_rad,
        double* armgoal_anglesV_rad,
        int numofDOFs,
        double*** plan,
        int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

    // ofstream myfile('foo.csv');
    // int vsize = returns.size();
    // for (int n=0; n<vsize; n++)
    // {
    //     myfile << returns[n] << endl;
    // }

    if(!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) || !IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size))
    {
        mexPrintf("invalid start or goal\n");
        return;
    }

    int max_samples = 200000, num_samples=0;
    bool found_tree = false;
    vector<double*> planned_path;
    
    clock_t tStart = clock();

    RRTree rrt(numofDOFs);

    rrt.init(armstart_anglesV_rad, armgoal_anglesV_rad);
    unordered_map<int,double> cost;
    cost[0]= 0.0;
    double gNew = 0.0;
    int goal_id=0;
    int valid_lb = 0;
    double first_path = -1.0;
    vector<double> cost_trend;
    vector<double> timesteps;
    double plan_cost = INT_MAX;
    // build the tree
    while(num_samples++ < max_samples)
    {
        // mexPrintf("found_goal: %d\n", found_tree);
        double* rand_angle = random_sample(numofDOFs, armgoal_anglesV_rad, 0.8);
        // double* rand_angle = rrt.sample_rrts(numofDOFs, armgoal_anglesV_rad, 0.8, 0.8, found_tree);

        // if(!found_tree or !rrt.node_reject(plan_cost, rand_angle))
        // {
            // extend
            unordered_set<int> neighbors = get_neighbors(rand_angle, rrt.vertices, 1, numofDOFs);
            // mexPrintf("main 3\n");
            double* qnearest = rrt.vertices[*neighbors.begin()].joints;
            // mexPrintf("main 4\n");
            double* qnew = new_config(qnearest, rand_angle, map, x_size, y_size, numofDOFs, 0.6);
            // mexPrintf("main 5\n");
            if(qnew!=NULL)
            {           
                vertex v = {rrt.get_numofvertices(), qnew};
                vertex qmin = {*neighbors.begin(),qnearest};
                rrt.add_vertex(v);
                cost[v.id] = cost[qmin.id] + dist_joints(qnearest, qnew, numofDOFs);
                unordered_set<int> qnear = get_near(qnew, rrt.vertices, 1.5, numofDOFs);
                // mexPrintf("qnear _size = %d\n", qnear.size());

                // rewire qnear
                for(auto xnear: qnear){
                    if(connect(rrt.vertices[xnear].joints, qnew, map, x_size, y_size, numofDOFs))
                    {
                        gNew = cost[xnear] + dist_joints(rrt.vertices[xnear].joints, qnew, numofDOFs);
                        if(gNew < cost[v.id])
                        {

                            // mexPrintf("rewire 1\n");
                            qmin = {xnear, rrt.vertices[xnear].joints};
                            cost[v.id] = gNew;
                        }
                    }
                }
                rrt.grow_tree(v.id, qmin.id);
                // rewire other vertices in the ball
                for(auto xnear: qnear){
                    if(!same_config(rrt.vertices[xnear].joints, qmin.joints, numofDOFs))
                    {
                        if(connect(rrt.vertices[xnear].joints, qnew, map, x_size, y_size, numofDOFs) && cost[xnear] > cost[v.id] + dist_joints(rrt.vertices[xnear].joints, qnew, numofDOFs))
                        {
                            cost[xnear] = cost[v.id] + dist_joints(rrt.vertices[xnear].joints, qnew, numofDOFs);
                            // mexPrintf("rewire 2\n");
                            rrt.grow_tree(xnear,v.id);
                        }
                    }
                }

                if((double)(clock() - tStart)/CLOCKS_PER_SEC>time_limit) break;
    
                if(same_config(qnew, armgoal_anglesV_rad, numofDOFs)){
                    goal_id = v.id;
                    rrt.plan = getPathRRT(rrt.grid, goal_id, rrt.vertices);
                    plan_cost = get_cost(rrt.plan, numofDOFs);
                    if(!found_tree){
                        first_path = (double)(clock() - tStart)/CLOCKS_PER_SEC;
                        mexPrintf("initial path cost RRT*: %.3f\n", plan_cost);
                        mexPrintf("initial path vertices: %d\n", rrt.get_numofvertices());
                    }
                    found_tree = true;        
                }
            }
        // }
        // if(num_samples%100==0){
        //     if(found_tree){
        //         rrt.plan = getPathRRT(rrt.grid, goal_id, rrt.vertices);
        //         cost_trend.push_back(get_cost(rrt.plan, numofDOFs));
        //     }else{
        //         cost_trend.push_back(numeric_limits<double>::infinity());
        //     }
        //     timesteps.push_back((double)(clock() - tStart)/CLOCKS_PER_SEC);
        // }
    }
    mexPrintf("num_samples: %d\n", num_samples);
    mexPrintf("valid_lb: %d\n", valid_lb);
    mexPrintf("Time taken RRT* : %.3fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    mexPrintf("Time taken 1st solution RRT* : %.3fs\n", first_path);

    // ofstream myfile("co.csv");
    // for(int i=0; i<cost_trend.size(); i++){
    //     myfile << timesteps[i] << "," << cost_trend[i] << endl;
    // }
    // myfile.close();

    if(found_tree)
    {
        rrt.plan = getPathRRT(rrt.grid, goal_id, rrt.vertices);
        *planlength = rrt.plan.size();
        *plan = (double**) malloc(*planlength*sizeof(double*));
        for(int i = 0; i < *planlength; i++)
        {
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
            for(int j = 0; j < numofDOFs; j++)
            {
                (*plan)[i][j] = rrt.plan[rrt.plan.size()-i-1][j];
            }
        }
        plan_cost = get_cost(rrt.plan, numofDOFs);
        mexPrintf("final path cost RRT*: %.3f\n", plan_cost);
    }
    else
    {
        mexPrintf("ERROR: No path found!!!\n");
    }
    mexPrintf("num vertices RRT*: %d\n", rrt.get_numofvertices());
    rrt.vertices.clear();
    rrt.grid.clear();
    
    return;
}

static void PRMPlan(
        double*	map,
        int x_size,
        int y_size,
        double* armstart_anglesV_rad,
        double* armgoal_anglesV_rad,
        int numofDOFs,
        double*** plan,
        int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    bool found_path = false;

    if(saved_map.size()==0) prm.roadmap_built= false;
    else{
        if(saved_map.size()!=x_size || saved_map[0].size()!=y_size) prm.roadmap_built= false;
        for(int i = 1; i < x_size && prm.roadmap_built; i++)
        {
            for(int j = 1; j < y_size; j++)
            {
                if(saved_map[i][j]!=map[GETMAPINDEX(i,j,x_size,y_size)])
                {
                    prm.roadmap_built = false;
                    break;
                }
            }
        }
    }
    if(!prm.roadmap_built){
        saved_map = vector<vector<double>>(x_size, vector<double>(y_size, 0));
        for(int i = 1; i < x_size; i++)
        {
            for(int j = 1; j < y_size; j++)
            {
                saved_map[i][j] = map[GETMAPINDEX(i,j,x_size,y_size)];
            }
        }
    }

    if(!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) || !IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size))
    {
        mexPrintf("invalid start or goal\n");
        return;
    }

    vector<double*> planned_path;

    clock_t tStart = clock();

    if(!prm.roadmap_built){
        mexPrintf("building roadmap\n");
        prm.init(armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs);
        prm.build_roadmap(map, x_size, y_size, 200000);
    }
    mexPrintf("roadmap built\n");
    prm.found_goal = prm.connect_ends(armgoal_anglesV_rad, armstart_anglesV_rad, map,x_size,y_size);
    // mexPrintf("1\n");
    if(!prm.found_goal) prm.found_goal = prm.find_ends(armgoal_anglesV_rad, armstart_anglesV_rad, map,x_size,y_size);
    // mexPrintf("2\n");
    if(prm.found_goal)
    {
        // grid, closed and opened hmap with key = v.id     
        unordered_map<int, cell> grid;
        unordered_map<int, bool> closed;
        unordered_map<int, bool> opened;

        // pq <pair(f, v.id)>
        priority_queue<listPair, vector<listPair>, greater<listPair>> open;    

        double max_dist = numeric_limits<double>::infinity();
        for(int i=0;i<prm.get_numofvertices();i++)
        {   
            cell c = {};
            c.parent = -1;
            c.g = max_dist;
            grid[i] = c;
            closed[i] = false;
            opened[i] = false;
        }
        mexPrintf("2..5\n");

        vertex start = prm.get_vertex(prm.start_id);
        cell c = {};
        c.g = 0;
        c.parent = prm.start_id;
        grid[prm.start_id] = c;
        open.push(make_pair(0.0, prm.start_id));
        // mexPrintf("3\n");
        while(!open.empty())
        {
            int vid = open.top().second;
            // mexPrintf("4\n");
            open.pop();
            if(closed[vid])
                continue;
            closed[vid] = true;
            vertex v = prm.get_vertex(vid);
            // mexPrintf("5\n");
            if(v.id == prm.goal_id){
                found_path = true;
                mexPrintf("Time taken PRM : %.3fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
                planned_path = getPath(grid, vid, prm.vertices);
                break;
            }
            // mexPrintf("6\n");
            for(auto s : prm.get_sucessors(v.id)){
                if(!closed[s])
                {
                    double gNew = grid[v.id].g + prm.get_edge_weight(v.id, s);
                    if(gNew < grid[s].g || grid[s].g == INT_MAX)
                    {
                        grid[s].g = gNew;
                        grid[s].parent = v.id;
                        open.push(make_pair(gNew, s));
                        opened[s] = true;
                    }
                }
                // mexPrintf("7\n");
            }        
        }
        // mexPrintf("8\n");
        if(found_path){
            *planlength = (int)planned_path.size();
            *plan = (double**) malloc(*planlength*sizeof(double*));
            for(int i=0;i<planned_path.size();i++)
            {
                (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
                for(int j=0;j<numofDOFs;j++)
                {
                    (*plan)[i][j] = planned_path[planned_path.size()-i-1][j];
                }
            }
            double plan_cost = get_cost(planned_path, numofDOFs);
            mexPrintf("path cost PRM: %.3f\n", plan_cost);
        }
        else
        {
            mexPrintf("no path found\n");
        }
    }
    else
    {
        mexPrintf("ERROR: No path found!!!\n");
    }
    mexPrintf("num vertices PRM: %d\n", prm.get_numofvertices());
    
    return;
}

////////////////////////////////////////////////////// END ///////////////////////////////////////////////////


//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    //you can may be call the corresponding planner function here
    if (planner_id == RRT)
    {
       RRTPlan(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if(planner_id == PRM)
    {
        PRMPlan(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if(planner_id == RRTCONNECT)
    {
        RRTConnectPlan(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if(planner_id == RRTSTAR)
    {
        RRTStarPlan(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else{
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    //dummy planner which only computes interpolated path
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return; 
}
