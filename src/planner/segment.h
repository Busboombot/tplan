#pragma once

#include <utility>
#include <vector>
#include <iostream>

#include "util.h"
#include "block.h"
#include "types.h"
#include "joint.h"
#include "planner_types.h"

#include "stepper.h"


using namespace std;

#ifdef TRJ_ENV_HOST
#include "json.hpp"
using json = nlohmann::json;
#else
using json = string;
#endif

class Planner;
class SegmentStepper;

/* Segment: One move for all joints, with Accel, Cruise and Decel phases. 
 * 
 *
 */

class Segment {

private:

    uint32_t n;
    const vector<Joint>& joints;
    trj_float_t t;
    MoveType moveType = MoveType::none;

    vector<Block> blocks;

    MoveVector moves;

    u_long n_joints;

public:
    const vector<Joint> &getJoints() const;

    Segment(uint32_t n, const std::vector<Joint>&  joints_);
    Segment(uint32_t n, const std::vector<Joint>&  joints_, MoveVector moves );
    Segment(uint32_t n, const std::vector<Joint>&  joints_, const Move& move );

    /**
     * @brief Segment constructor with max velocity vector, for JOG and VMOVE moves.
     * @param n
     * @param joints_
     * @param moves
     * @param maxv
     */
    Segment(uint32_t n, const std::vector<Joint>&  joints_, MoveVector moves, MoveVector maxv );

    /**
     * @brief Plan all of the blocks in this segment
     * @param t_
     * @param v_0_
     * @param v_1_
     * @param prior
     * @param next
     */
    void plan(trj_float_t t_=NAN, int v_0_=0, int v_1_=0, Segment *prior = nullptr, Segment *next = nullptr);

    /**
     * @brief Plan all of the blocks in this segment as velocity based moves
     * @param t_
     * @param prior
     * @param next
     */
    void vplan(trj_float_t t_=NAN, Segment *prior = nullptr, Segment *next = nullptr);

    void setBv(int v_0, int v_1);

    void setBv(vector<int> v_0_, vector<int> v_1_);

public:

    MoveType getMoveType() const;

    const MoveVector& getMoves() const {return moves;}

    VelocityVector getV0();
    VelocityVector getV1();
    int getN() const { return n; }

    trj_float_t getT() const;

    /**
     * @brief Calculated maximum of the minimum times of the blocks
     * @return
     */
    trj_float_t calcMinTime();

    /**
     * @brief Maximum of stored time of the blocks
     * @return
     */
    trj_float_t maxBlockTime();

    trj_float_t time();

    trj_float_t timeErr(); // RMS difference in times of blocks

    json dump(std::string tag="", bool dump_joints=false) const;

    static trj_float_t boundaryError(const Segment& prior, const Segment &next);

    friend Planner;
    friend SegmentStepper;
    friend ostream &operator<<( ostream &output, const Segment &s );

};