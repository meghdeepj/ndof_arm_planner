#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <math.h>
#include <unordered_set>
#include <queue>
#include <limits>
#include <stack>
#include <time.h>
#include "mex.h"

using namespace std;

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

struct vertex
{
    int id;
    double* joints;
};


typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
  double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
  double x0,y0,x1,y1;
  int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
  y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
    return 1;
}

double get_norm(double* x,int size)
{
  double  total = 0.0;
  for (int i = 0; i < size; i++)
  {
    total += x[i]*x[i];
  }
  return sqrt(total);
}

double dist_joints(double* a1, double* a2, int numofDOFs){
  double dist = 0;
  for(int i = 0; i < numofDOFs; i++){
    dist += pow(a1[i]-a2[i],2);
  }
  return sqrt(dist);
}

unordered_set<int> get_near(double* q, vector<vertex> rmap, double r, int n){
  unordered_set<int> near;
  for(int i = 0; i < rmap.size(); i++){
    if(dist_joints(q, rmap[i].joints, n) < r){
      near.insert(rmap[i].id);
    }
  }
  return near;
}

// K N N
unordered_set<int> get_neighbors(double* v_angle, vector<vertex> rmap, int k, int n){
  unordered_set<int> neighbors;
  k = min(k, (int)rmap.size());
  if(rmap.size()==1)
  {
      neighbors.insert(0);
      return neighbors;
  }
  while(k--){
    int min_id = -1;
    double min_dist = numeric_limits<double>::infinity();
    // mexPrintf("size: %d\n", rmap.size());
    for(int i = 0; i < rmap.size(); i++){
      // mexPrintf("iter %d\n", i);
      if(neighbors.count(i)==1) continue;
      double dist = dist_joints(v_angle, rmap[i].joints, n);
      // mexPrintf("i %d\n", i);
      if(dist < min_dist){
        min_dist = dist;
        min_id = rmap[i].id;
      }
    }
    neighbors.insert(min_id);

  }
  return neighbors;
}

double* random_sample(int nDoF, double* goal_pose, double prob){
  double* rand_angle = new double[nDoF];
  for( int i=0;i<nDoF;i++)
  {
    rand_angle[i] = ((double)(rand()%360))*PI/180.0;
  }
  double explore_prob = (double)rand()/(RAND_MAX+1.0);
  return explore_prob<=prob ? rand_angle: goal_pose;
}

double* new_config(double* v, double* q, double* map, int x_size, int y_size, int nDoF, double eps){
  // check if connection possible via interpolation
  // get vector interpolation b/w v and q
  // check if config is valid for each interpolated point
  // mexPrintf("new_conf 3\n");
  double dist = 0;
  for (int i = 0; i < nDoF; i++)
  {
    if(dist < abs(v[i] - q[i]))
      dist = abs(v[i] - q[i]);
  }
  int k = (int)(dist/(PI/40.0));
  // mexPrintf("new_conf k: %d\n", k);
  double* temp = new double[nDoF];
  double* ret_angle = new double[nDoF];
  bool advanced = false;
  // mexPrintf("k %d\n", k);


  for(int i = 1; i < k; i++){
    for(int j = 0; j < nDoF; j++){
      temp[j] = v[j] + (q[j]-v[j])*(((double)i)/(k-1));
    }
    // mexPrintf("new_conf 5\n");
    if(IsValidArmConfiguration(temp, nDoF, map, x_size, y_size) && dist_joints(temp, v, nDoF) <= eps)
    {
      ret_angle=temp;
      advanced = true;
    }else{
      break;
    }
    
  }
  return advanced? ret_angle:NULL;
}

bool same_config(double* a, double* b, int nDoF){
  if(a == NULL || b == NULL){
		// mexPrintf("sameconf1\n");
    return false;
  }
  for(int i = 0; i < nDoF; i++){
    if(a[i] != b[i]){
      // mexPrintf("sameconf2\n");
      return false;
    }
  }
  // mexPrintf("sameconf3\n");
  return true;
}


bool connect(double* v, double* q, double* map, int x_size, int y_size, int nDoF){
  // check if connection possible via interpolation
  // get vector interpolation b/w v and q
  // check if config is valid for each interpolated point
  double dist = 0;
  for (int i = 0; i < nDoF; i++)
  {
    if(dist < abs(v[i] - q[i]))
      dist = abs(v[i] - q[i]);
  }
  int k = (int)(dist/(PI/40.0));
  double* temp = new double[nDoF];
  double* ret_angle = new double[nDoF];
  for(int i = 1; i < k; i++){
    for(int j = 0; j < nDoF; j++){
      temp[j] = v[j] + (q[j]-v[j])*(((double)i)/(k-1));
    }
    if(!IsValidArmConfiguration(temp, nDoF, map, x_size, y_size))
    {
      return false;
    }
  }
  return true;
}

bool goal_reached(double* q1, double* qgoal, int nDoF, double thresh){
  double distance = dist_joints(q1, qgoal, nDoF);
  
  if(distance < thresh)
  {
    mexPrintf("goal dist: %.3f \n", distance);
    return true;
  }
  return false;
}

class RRTree
{ 
  public:
    RRTree(int numDof)
    {
      n = numDof;
      vertices.clear();
      grid.clear();
      plan.clear();
    }

    void init(double *start_state, double* goal_state)
    {
      sample_id = 0;     
      vertex v = {0, start_state};
      add_vertex(v);
      start = start_state;
      goal = goal_state;
    }

    bool node_reject(double plan_cost, double* qs)
    {
        if(dist_joints(qs, start, n) + dist_joints(qs, goal, n) > plan_cost)
        {
          // mexPrintf("node reject\n");
          return true;
        }
        return false;
    }

    double* local_bias(vector<double*> plan)
    {
      // mexPrintf("local_bias\n");
      int len = plan.size();
      int rand_id = 2+ rand()%(len-4) ;
      double* q = plan[rand_id];
      double* q2 = plan[rand_id-1];
      double* q1 = plan[rand_id+1];
      double* q_tmp = new double[n];
      for(int i = 0; i < n; i++){
        q_tmp[i]= (q1[i]+q2[i])/2.0-q[i];
      }
      double rand_r = 0.5+ (double)(rand()%100)*0.01;
      double* q_rand = new double[n];
      for(int i = 0; i < n; i++){
        q_rand[i]= q[i] + (q_tmp[i]/get_norm(q_tmp, n))*rand_r;
      }
      return q_rand;
    }

    double* sample_rrts(int nDoF, double* goal_pose, double rand_prob, double bias_prob, bool path_found){
      double* rand_angle = new double[nDoF];
      for( int i=0;i<nDoF;i++)
      {
        rand_angle[i] = ((double)(rand()%360))*PI/180.0;
      }
      double explore_prob = (double)rand()/(RAND_MAX+1.0);
      if(explore_prob>rand_prob && !path_found)
        return goal_pose;
      else if(explore_prob>bias_prob && path_found)
        return local_bias(plan);
      else
        return rand_angle;
    }

    void add_vertex(vertex v)
    {
      vertices.push_back(v);
    }

    void grow_tree(int vid1, int vid2){
      grid[vid1] = vid2;
    }

    vertex get_vertex(int id)
    {
      return vertices[id];
    }

    int get_numofvertices()
    {
      return vertices.size();
    }

    int get_sucessors(int id)
    {
      return grid[id];
    }

    double get_edge_weight(int id1, int id2)
    {
      dist_joints(vertices[id1].joints, vertices[id2].joints, n);
    }

    vector<vertex> vertices;
    unordered_map<int,int> grid;
    vector<double*> plan;

  private:
    int sample_id;
    int total_samples;    
    int n;
    double* start;
    double* goal;

};

class PRoadmap
{ 
  public:
    void init(double *start_state, double* goal_state, int numDof)
    {
      roadmap_built = false;
      n = numDof;
      vertices.clear();
      edges.clear();
      component.clear();
      found_goal = false;
      goal_id = -1;
      start_id = 0;
      sample_id = 0;     
      vertex v = {0, start_state};
      add_vertex(v);
      start = start_state;
      goal = goal_state;
    }

    bool same_component(vertex v1, vertex v2){
      return component[v1.id] == component[v2.id];
    }

    void add_vertex(vertex v)
    {
      if(component.find(v.id) == component.end())
      {
        component[v.id] = sample_id;
        vertices.push_back(v);
        edges.push_back(vector<int>());
      }
    }

    void add_edge(vertex v1, vertex v2){
      int c1 = component[v1.id];
      for(auto it : component)
      {
        if(it.second == c1)
        {
          component[it.first] = component[v2.id];
        }
      }
      edges[v1.id].push_back(v2.id);
      edges[v2.id].push_back(v1.id);
    }

    vertex get_vertex(int id)
    {
      return vertices[id];
    }

    vector<int> get_sucessors(int id)
    {
      return edges[id];
    }

    int get_numofvertices()
    {
      return vertices.size();
    }

    double get_edge_weight(int id1, int id2)
    {
      dist_joints(vertices[id1].joints, vertices[id2].joints, n);
    }

    bool find_ends(double* endpt, double* startpt, double* map, int x_size, int y_size){
      bool found_start = false;
      found_goal = false;
      start_id=0;
      for(int i = 0; i < get_numofvertices(); i++)
      {
        found_start = same_config(vertices[i].joints, startpt, n);
        if(found_start)
        {
          start_id = i;
          break;
        }
      }

      for(int i = 0; i < get_numofvertices(); i++)
      {
        found_goal = same_config(vertices[i].joints, endpt, n);
        if(found_goal)
        {
          goal_id = i;
          break;
        }
      }
      if(found_goal && found_start && same_component(get_vertex(goal_id),get_vertex(start_id)))
        return true;
    }

    bool connect_ends(double* endpt, double* startpt, double* map, int x_size, int y_size)
    {
      bool found_start = false;
      found_goal = false;
      start_id=0;
      for(int i = 0; i < get_numofvertices(); i++)
      {
        found_start = same_config(vertices[i].joints, startpt, n);
        if(found_start)
        {
          start_id = i;
          break;
        }
      }

      for(int i = 0; i < get_numofvertices(); i++)
      {
        found_goal = same_config(vertices[i].joints, endpt, n);
        if(found_goal)
        {
          goal_id = i;
          break;
        }
      }
      if(found_goal && found_start && same_component(get_vertex(goal_id),get_vertex(start_id)))
        return true;
      goal = endpt;
      sample_id++;
      if(!found_start){
        // mexPrintf("add start\n");
        start = startpt;
        sample_id++;
        vertex vs = {get_numofvertices(), startpt};
        add_vertex(vs);
        unordered_set<int> neighbors;
        neighbors = get_neighbors(vs.joints, vertices, 20,n);
        for(int id : neighbors)
        {
          vertex q = get_vertex(id);
          if(!same_component(vs,q) && connect(vs.joints,q.joints,map,x_size,y_size,n))
          {
            add_edge(vs,q);
          }
        }
        start_id = vs.id;
      }
      if(!found_goal){
        // mexPrintf("add goal\n");
        vertex vg = {get_numofvertices(), endpt};
        unordered_set<int> neighbors;
        add_vertex(vg);
        neighbors = get_neighbors(vg.joints, vertices, 20,n);
        // mexPrintf("5: %d\n",neighbors.size());
        for(int id : neighbors)
        {
          vertex q = get_vertex(id);
          if(!same_component(vg,q) && connect(vg.joints,q.joints,map,x_size,y_size,n))
          {
            add_edge(vg,q);
            if(same_component(vg,get_vertex(start_id))){
              goal_id = vg.id;
              return true;
            }
          }else{
            return false;
          }
        }
      }

    }

    void build_roadmap(double* map, int x_size, int y_size, int samples)
    {
      int i,j;
      total_samples = samples;
      vertex v;
      unordered_set<int> neighbors;
      double* rand_angle = new double[n];
      while(sample_id < total_samples)
      {
        sample_id++;
        // mexPrintf("num_samples: %d \n", sample_id);
        rand_angle = random_sample(n, goal,1.1);
        if(IsValidArmConfiguration(rand_angle, n, map, x_size, y_size))
        {
          v = {get_numofvertices(), rand_angle};
          add_vertex(v);
          neighbors = get_neighbors(v.joints, vertices, 5,n);
          for(int id : neighbors)
          {
            vertex q = get_vertex(id);
            if(!same_component(v,q) && connect(q.joints,v.joints,map,x_size,y_size,n))
            {
              add_edge(v,q);
              if(same_config(rand_angle,goal, n) && same_component(v,get_vertex(0))){
                goal_id = v.id;
              }
            }
          }
        }
      }
      roadmap_built = true;
    }
    vector<vertex> vertices;
    bool found_goal, roadmap_built;
    int goal_id, start_id;

  private:
    int sample_id;
    int total_samples;
    unordered_map<int,int> component;
    vector<vector<int>> edges;
    int n;
    double* start;
    double* goal;

};