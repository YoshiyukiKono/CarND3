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
#include <memory>
#include "vehicle.h"
#include "DrivingStrategy.h"

using namespace std;

class DrivingStrategy;

class BehaviorPlanner {
public:

	const int SAFE_DISTANCE = 20;
	double fore_s_right;
	double fore_s_vehicle;
	
	bool too_close;
	int cur_lane;
	int target_lane;
	bool lane_change;

	int recommended_lane;
	double faster_lane_speed;
	double farther_lane_s;
	double ego_lane_speed;

	double farther_L_s;
	double faster_L_v;
	double farther_R_s;
	double faster_R_v;

	double L_vehicle_num;
	double R_vehicle_num;

  	int num_lanes;

    map<int, Vehicle> vehicles;
    Vehicle ego_vehicle;

    int vehicles_added = 0;

    /**
  	* Constructor
  	*/
  	BehaviorPlanner();

  	/**
  	* Destructor
  	*/
  	virtual ~BehaviorPlanner();

  	void populate_traffic(vector<vector<double>> sensor_fusion);

  	void plan(int& lane, int& prev_size);

  	void add_ego(int lane_num, int s, double speed);

	bool is_safe_to_move(int lane_num);
	void init_recommend_lane();
	int recommend_lane();
	bool is_initialized();

	void update_route_planning_offline();

	// Unimplemented training function
	void train(vector<vector<double>> sensor_fusion);

private:

	map<int, shared_ptr<DrivingStrategy>> drive_strategies;

	int convert_to_lane(float d);

	// Unimplemented prediction functions
	void predict();
	void predict_vehicle(Vehicle& vehicle);
	void save_previous_state();
	void update_prediction_model(int lane, int prev_lane, float s, float d, float s_dot, float d_dot);
	int get_prev_lane(int vehicle_id);

	// Unimplemented cost functions
	bool is_better_to_move();
	int recommend_lane_in_a_long_run();
	void avoid_if_dangerous_situation();
};


#endif