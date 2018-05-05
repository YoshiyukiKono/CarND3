#ifndef BAHAVE_H
#define BAHAVE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class BehaviorPlanner {
public:

	const int SAFE_DISTANCE = 20;
	double fore_s_right;
	double fore_s_vehicle;

	int recommended_lane;
	double faster_lane_speed;
	double farther_lane_s;
	double ego_lane_speed;
	double ego_lane_s;
	double farther_L_s;
	double faster_L_v;
	double farther_R_s;
	double faster_R_v;

	double L_vehicle_num;
	double R_vehicle_num;
	
	int update_width = 70;

  	string ego_rep = " *** ";

  	int ego_key = -1;

  	int num_lanes;

    vector<int> lane_speeds;

    int speed_limit;

    double density;

    int camera_center;

    map<int, Vehicle> vehicles;
    Vehicle ego_vehicle;

    int vehicles_added = 0;

    /**
  	* Constructor
  	*/
  	BehaviorPlanner(int speed_limit, double traffic_density, vector<int> lane_speeds);
  	BehaviorPlanner();

  	/**
  	* Destructor
  	*/
  	virtual ~BehaviorPlanner();

  	Vehicle get_ego();

  	void populate_traffic();
  	void populate_traffic(vector<vector<double>> sensor_fusion);

  	void advance();

  	void display(int timestep);

  	void add_ego(int lane_num, int s, vector<int> config_data);
  	void add_ego(int lane_num, int s, double speed);

  	void cull();
  	
  	int convert_to_lane(float d);
  	bool is_safe_to_move(int lane_num);
  	bool is_better_to_move();
  	bool is_better_to_move(int lane_num);
  	int recommend_lane();
  	bool is_risky_to_move(double back_vehicle_v);

	void find_vehicle_forward(int lane_num, int s);
	void find_vehicle_backward(int lane_num, int s);
	
	void init_recommend_lane();
};
#endif