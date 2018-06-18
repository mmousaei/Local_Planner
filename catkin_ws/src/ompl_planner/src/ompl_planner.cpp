#include"ompl_planner.h"

Planner::Planner(){};

bool Planner::isStateValid(const ob::State *state)
{
   // cast the abstract state type to the type we expect
   const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

   // extract the first component of the state and cast it to what we expect
   const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

   // extract the second component of the state and cast it to what we expect
   const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

   // check validity of state defined by pos & rot


   // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
   return (const void*)rot != (const void*)pos;
}

void Planner::plan(nav_msgs::Odometry start_nav, nav_msgs::Odometry goal_nav)
{
   // construct the state space we are planning in
   auto space(std::make_shared<ob::SE3StateSpace>());

   // set the bounds for the R^3 part of SE(3)
   ob::RealVectorBounds bounds(3);
   bounds.setLow(-1);
   bounds.setHigh(1);

   space->setBounds(bounds);

   // construct an instance of  space information from this state space
   auto si(std::make_shared<ob::SpaceInformation>(space));

   // set state validity checking for this space
   si->setStateValidityChecker(isStateValid);

   // create a random start state
   ob::ScopedState<> start(space);
   start.random();

   // create a random goal state
   ob::ScopedState<> goal(space);
   goal.random();

   // create a problem instance
   auto pdef(std::make_shared<ob::ProblemDefinition>(si));

   // set the start and goal states
   pdef->setStartAndGoalStates(start, goal);

   // create a planner for the defined space
   auto planner(std::make_shared<og::RRTConnect>(si));

     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);

     // perform setup steps for the planner
     planner->setup();


     // print the settings for this space
     si->printSettings(std::cout);

     // print the problem settings
     pdef->print(std::cout);

     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Found solution:" << std::endl;

         // print the path to screen
         path->print(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
 }

 void Planner::planWithSimpleSetup(nav_msgs::Odometry start_nav, nav_msgs::Odometry goal_nav)
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE3StateSpace>());

     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-1);
     bounds.setHigh(1);

     space->setBounds(bounds);

     // define a simple setup class
     og::SimpleSetup ss(space);

     // set state validity checking for this space
     ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

     // create a random start state
     ob::ScopedState<> start(space);
     // start.random();
     start->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(start_nav.pose.pose.position.x,start_nav.pose.pose.position.y,start_nav.pose.pose.position.z);
     start->as<ompl::base::SE3StateSpace::StateType>()->rotation().setAxisAngle(start_nav.pose.pose.orientation.x, start_nav.pose.pose.orientation.y, start_nav.pose.pose.orientation.z, start_nav.pose.pose.orientation.w);
     std::cout << "start = " <<start<< '\n';

     // create a random goal state
     ob::ScopedState<> goal(space);
     // goal.random();
     goal->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(goal_nav.pose.pose.position.x,goal_nav.pose.pose.position.y,goal_nav.pose.pose.position.z);
     goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().setAxisAngle(goal_nav.pose.pose.orientation.x, goal_nav.pose.pose.orientation.y, goal_nav.pose.pose.orientation.z, goal_nav.pose.pose.orientation.w);
     std::cout << "goal = " <<goal<< '\n';

     // set the start and goal states
     ss.setStartAndGoalStates(start, goal);

     // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();

     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = ss.solve(1.0);

     if (solved)
     {
         std::cout << "Found solution:" << std::endl;
         // print the path to screen
         ss.simplifySolution();
         ss.getSolutionPath().print(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
 }

 // int main(int /*argc*/, char ** /*argv*/)
 // {
 //     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
 //
 //     plan();
 //
 //     std::cout << std::endl << std::endl;
 //
 //     planWithSimpleSetup();
 //
 //     return 0;
 // }
