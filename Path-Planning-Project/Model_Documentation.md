# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Model Documentation

### Steps that I took

To achieve this project, I took the following steps.

* I started from the simplest implementation that makes the car drives without incidents.
* Then, I refactored the first version by breaking code apart into logical pieces.
* Finally, I applied the design that I learned from the lessons even though some of the components look useless for this project,
so that I obtain the insight about the path-planning in the real world, and have the placeholders that I'm going to write advanced logics, 
although it wouldn't change the car's behavior such drastically in the simulator.

### What I came up with

From the above process, I ended up with thinking about the cost function as follows.

* It might be better to treat the cost at some different levels rather than having the only one consolidated cost, that is,
1) simple(low-computing-cost) one for the decision taken at the time, which is almost enough for this project
2) more complex one for the route in a long run, for which prediction works to find better route
3) emergency one for the exceptional dangerous situation

### Design of my project

I'll explain my design and implementation in comparison to the design that provided by the lessons.

First, I put relevant logics into BehaviorPlanner class.

#### Route Planning

In the overview diagram provided by the lesson, Route Planning is located offline, apart from the other three components that has online data flow.
Although the simulator has just one situation, running on the highway that has three lanes, 
I designed DrivingStrategy class to express the scene where road situation is changed.
Certainly, I admit the abstraction level of my code is not enough though, I intended to clarify the point.

#### Prediction

As implied in "What I came up with", I omitted to implement prediction process. Instead I put some skeleton functions.

#### Behavior Planning

I implemented the simple logic mentioned above. However, I also wrote some skeleton functions for exceptional situation.
In these unimplemented functions, I left the comment about what logics can be added there.

#### Trajectory Planning

As for trajectory planning, I applied the algorithm introduced Q&A session. What I did a little is to break code into some functions.
I left the functions in main.cpp as it depends on the existing functions defined there.

