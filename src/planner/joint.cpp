#include <array>
#include <iostream>
#include "joint.h"


std::ostream &operator<<(std::ostream &output, const Joint &j) {

    output << "[J " << j.n << " v=" << j.v_max << " a=" << j.a_max  << " ]";
    return output;

}

#ifdef TRJ_ENV_HOST
json Joint::dump() const{

    json j;
    j["_type"] = "Joint";
    j["n"] = n;
    j["v_max"] = v_max;
    j["a_max"] = a_max;

    return j;
}
#else
json Joint::dump() const{
    return string("");
}
#endif