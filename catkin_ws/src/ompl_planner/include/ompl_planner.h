/*    eop_estimation.cpp - class definition of implementation of end of
 *                         pipe detection algorithms
 *
 *    Carnegie Mellon University
 *    Author: Mohammadreza Mousei
 *    Contact: mmousaei@andrew.cmu.edu
 *
 */

#ifndef OMPL_PLANNER_H
#define OMPL_PLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <nav_msgs/Odometry.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner {

public:
    Planner();
    void plan(nav_msgs::Odometry start_nav, nav_msgs::Odometry goal_nav);
    void planWithSimpleSetup(nav_msgs::Odometry start_nav, nav_msgs::Odometry goal_nav);
    static bool isStateValid(const ob::State *state);
private:

    int alaki;
};


#endif //OMPL_PLANNER_H
