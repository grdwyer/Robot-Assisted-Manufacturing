//
// Created by george on 3/30/21.
//

#ifndef RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
#define RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H

#include <ram_motion_planning/base_toolpath_planner.h>

class OMPLToolpathPlanner : public BaseToolpathPlanner{
    using BaseToolpathPlanner::BaseToolpathPlanner;
    bool construct_plan_request();
};

#endif //RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
