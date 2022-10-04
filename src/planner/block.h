#pragma once

#include <tuple>
#include <stdexcept>
#include <iostream>
#include <map>
#include <array>


#include "util.h"
#include "planner_types.h"

#ifdef TRJ_ENV_HOST

#include "json.hpp"

using json = nlohmann::json;
#else
using json = string;
#endif

using namespace std;
using std::ostream;

struct StepperPhase;

class Joint;

class Planner;

/**
 * @brief Blocks are the fundamental unit of the planner, representing one move for one axis
 */
class Block {

public:

    Block(trj_float_t x, const Joint &joint);

    Block(trj_float_t x, trj_float_t v_0, trj_float_t v_1, const Joint &joint);

    /**
     * @brief Plan the parameters of this block, given a distance and a time
     * @param t_ Target time for the block
     * @param v_0_ Specification of the initial velocity
     * @param v_1_ Specification of the final velocity
     * @param prior Link to the block from the same axis in the previous segment
     * @param next  Link to the block from the same axis in the next segment.
     */
    void plan(trj_float_t t_ = NAN, int v_0_ = BV_NAN, int v_1_ = BV_NAN,
              Block *prior = nullptr, Block *next = nullptr);

    /**
     * @brief Plan the parameters for the block, given a target time and a velocity.
     * @param t_ Target time
     * @param prior Link to the block from the same axis in the previous segment
     * @param next  Link to the block from the same axis in the next segment.
     */
    void vplan(trj_float_t t_, Block *prior = nullptr, Block *next = nullptr);

    trj_float_t area();

    trj_float_t getT() const;

    trj_float_t calcMinTime() const;

    void setBv(int v_0_, int v_1_, Block *prior = nullptr,
               Block *next = nullptr); // Clip the boundary values based on the distance

    void limitBv();

    trj_float_t getV0() const;

    trj_float_t getV1() const;

    json dump(std::string tag = "") const;

    array<StepperPhase, 3> getStepperPhases() const;

    static bool bent(Block &prior, Block &current);

    static trj_float_t meanBv(Block &prior, Block &next);

    friend class Planner;

    friend class Segment;

    friend ostream &operator<<(ostream &output, const Block &s);

private:
    trj_float_t x;
    trj_float_t d = 0;
    trj_float_t t = 0;

    trj_float_t t_a = 0;
    trj_float_t t_c = 0;
    trj_float_t t_d = 0;

    trj_float_t x_a = 0;
    trj_float_t x_c = 0;
    trj_float_t x_d = 0;

    trj_float_t v_0 = 0;
    trj_float_t v_c = 0;
    trj_float_t v_1 = 0;

    // Max velocity for v_c, used for velocity moves

    const Joint &joint;

    void set_zero();

    std::tuple<trj_float_t, trj_float_t>
    accel_xt(trj_float_t v_i, trj_float_t v_1) const; // Compute trapezoid for acceleration from v_i to v_1

    std::tuple<trj_float_t, trj_float_t>
    accel_acd(trj_float_t v_0_, trj_float_t v_c_, trj_float_t v_1_) const; // Compute both accel and decel trapezoids

protected:
    trj_float_t v_c_max;

};