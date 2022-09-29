#include <iostream>
#include <vector>

#include "test.h"
#include "joint.h"
#include "planner.h"


#include <catch2/catch_test_macros.hpp>



TEST_CASE("Basic Planner Test", "[planner]"){

    vector<Joint> joints = get2Joints();

    Planner p(joints);

    p.move(1,{1000, 1000});
    p.move(2,{1000, 1000});
    p.move(3,{1000, 1000});


    cout <<" ============ " << endl;
    cout << p << endl;
}

TEST_CASE("Large Small Planner Test", "[planner]"){

    vector<Joint> joints = get2Joints();

    Planner p(joints);

    p.move(10,{1000, 1});
    p.move(11,{1, 1000});

    cout <<" ============ " << endl;
    cout << p << endl;

}




