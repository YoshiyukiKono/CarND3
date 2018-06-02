#ifndef STRATEGY_H
#define STRATEGY_H

#include "BehaviorPlanner.h"

class BehaviorPlanner;

struct DrivingStrategy {
	virtual void adopt(BehaviorPlanner* bp) = 0;
	virtual ~DrivingStrategy() {}
};
class DrivingStrategyRightEnd : public DrivingStrategy{
	void adopt(BehaviorPlanner* bp);
};
class DrivingStrategyLeftEnd : public DrivingStrategy{
	void adopt(BehaviorPlanner* bp);
};
class DrivingStrategyMiddle : public DrivingStrategy{
	void adopt(BehaviorPlanner* bp);
};

#endif