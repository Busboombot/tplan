
#pragma once

#include <cstdint> 
#include <deque>
#include <vector>
#include <array>
#include <iostream>
#include <iomanip>
#include <cmath> // rint
#include <initializer_list> 


#include "const.h" // For N_AXES
#include "util.h"
#include "types.h"
#include "joint.h"
#include "planner_types.h"
#include "stepper.h"
#include "segment.h"

#ifdef TRJ_ENV_HOST
#include "json.hpp"
using json = nlohmann::json;
#else
using json = string;
#endif


using namespace std;

class Joint;
class SegmentStepper;

class Planner {


public:

    explicit Planner();
    explicit Planner(std::vector<Joint> joints);

    void setJoints(std::vector<Joint> joints);

    // Add a move, processing it into a Segment
    void move(const Move& move);
    void move(unsigned int seq_id, const MoveArray &move);

    void plan();

    bool isEmpty() { return segments.empty(); }
    unsigned long getNSegments(){  return segments.size();}
    Segment &getFront() { return segments.front(); }
    void popFront() { return segments.pop_front(); }

    uint32_t getQueueTime() const{ return queue_time;  }

    uint32_t getQueueSize() const{ return  segments.size(); }

    MoveArray getPosition(){ return planner_position; }

    void updateCurrentState(CurrentState &current_state);

    MoveType getCurrentMoveType(){
        if (!isEmpty()) {
            return segments.front().getMoveType();
        } else {
            return MoveType::none;
        }
    }


    const std::vector<Joint> &getJoints(){ return joints;}

    const Joint &getJoint(int i){ return joints[i];}

    const deque<Segment> &getSegments() const;



    json dump(const std::string& tag="") const;

    // Fpr passing in to set_bv for boundaries you don't want to change.
    VelocityVector V_NAN = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};

    friend ostream &operator<<( ostream &output, const Planner &p );

    friend SegmentStepper;

private:

    std::vector<Joint> joints; // Joint configuration

    std::deque<Segment> segments;

    int32_t queue_time=0;

    unsigned int seg_num = 0;

    MoveArray planner_position;
    MoveArray completed_position;

    double boundary_error(Segment &p, Segment &c);


};
