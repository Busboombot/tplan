#include "../test.h"
#include "json.hpp"

#include "segment.h"
#include "joint.h"
#include "planner_types.h"


using json = nlohmann::json;
using namespace std;



TEST_CASE("Low Level Block Test", "[block]")
{
    Joint j(0, 5e3, 50e3);
    Segment s(0,{j, j});


    {
        Block b(1000, 0, 0, j);
        b.plan();
        cout << "A " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(1000, 2500, 2500, j);
        b.plan();
        cout << "B " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(1000, 5000, 5000, j);
        b.plan();
        cout << "C " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(200, 5000, 5000, j);
        b.plan();
        cout << "D " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(1, 5000, 5000, j);
        b.plan();
        cout << "E " << b.calcMinTime() << " " << b << endl;
    }

    {
        Block b(1000, 5000, 0, j);
        b.plan();
        cout << "F " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(1000, 0, 5000, j);
        b.plan();
        cout << "G " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(400, 5000, 0, j);
        b.plan();
        cout << "H " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(400, 0, 5000, j);
        b.plan();
        cout << "I " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(150, 5000, 0, j);
        b.plan();
        cout << "J " << b.calcMinTime() << " " << b << endl;
    }
    {
        Block b(150, 0, 5000, j);
        b.plan();
        cout << "K " << b.calcMinTime() << " " << b << endl;
    }

}

// This version of the test uses the same block configuration
// as above, but outputs JSON so it can be compared to the Python version,
// in ACDBlock.ipynb
TEST_CASE("Low Level Block Test JSON", "[block][json]")
{
    Joint j(0, 5e3, 50e3);
    Segment s(0, {j, j});

    vector<Block> blocks = {
            Block(1000, 0, 0, j),       // A
            Block(1000, 2500, 2500, j), // B
            Block(1000, 5000, 5000, j), // C
            Block(200, 5000, 5000, j),  // D
            Block(1, 5000, 5000, j),    // E
            Block(1000, 5000, 0, j),    // F
            Block(1000, 0, 5000, j),    // G
            Block(400, 5000, 0, j),     // H
            Block(400, 0, 5000, j),     // I
            Block(150, 5000, 0, j),     // J
            Block(150, 0, 5000, j)      // K
    };

    vector<json> o;

    string tags = "ABCDEFGHIJK";
    string::iterator tag_i = tags.begin();
    string tag;
    for(Block &b: blocks){
        b.plan();
        tag.assign(1,*tag_i);
        o.push_back(b.dump(tag));
        tag_i++;
    }

    json jout;
    jout["test"] = "low level block";
    jout["output"] = o;

    cout << "JSON" << jout << endl;

}