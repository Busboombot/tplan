
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

    /**
     * @brief Add a move, processing it into a Segment
     * @param move
     */
    void move(const Move& move);
    void move(unsigned int seq_id, const MoveVector &move);

    /**
     * @brief Add a velocity move which specified velocity and time rather than distance.
     * @param move
     */
    void vmove(const Move& move);
    void vmove(unsigned int seq_id, trj_float_t t, const MoveVector &move);

    void jog(const Move& move);
    void jog(unsigned int seq_id, trj_float_t t, const MoveVector &move);

    void plan();

    bool isEmpty() { return segments.empty(); }
    unsigned long getNSegments(){  return segments.size();}
    Segment &getFront() { return segments.front(); }
    void popFront();

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

    MoveArray &getPlannerPosition()  {
        return planner_position;
    }

    MoveArray &getCompletedPosition()  {
        return completed_position;
    }

    const vector<Joint> &getJoints(){ return joints;}

    const Joint &getJoint(int i){ return joints[i];}

    vector<Segment> getSegments() const{
        vector<Segment> v(segments.begin(), segments.end());
        return v;
    }


    /**
     * @brief Truncate the segments queue to sz elements, from the start
     * @param sz Number of elements left after the operation
     */
    void truncateTo(size_t sz){
        while(segments.size() > sz){
            planner_position -= segments.back().moves;
            segments.pop_back();
        }
    }

    /**
     * @brief Reset the planner, deleting all of the segments
     *  and setting the planner position and completed position to zero.
     *  ( Actually truncates to 1, to let the current segment runout, if it is running
     *  or to 0 if it is stopped )
     */
    void reset(bool running);

    void zero();

    void setPositions(const MoveVector &mv);

    /**
     * @brief Return a JSON object representing the state of the planner
     * and it's segments.
     * @param tag Value to include in the '_tag' value in the dict
     * @return a JSON dict
     */
    json dump(const std::string& tag="") const;

    // Fpr passing in to set_bv for boundaries you don't want to change.
    VelocityVector V_NAN = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};

    friend ostream &operator<<( ostream &output, const Planner &p );

    friend SegmentStepper;

private:

    vector<Joint> joints; // Joint configuration

    deque<Segment> segments;

    int32_t queue_time=0;

    MoveArray planner_position = {0};
    MoveArray completed_position = {0};

};