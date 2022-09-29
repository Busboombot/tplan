

#include <cstdint> 
#include <iostream>
#include <iomanip>

#include "move.h"

using namespace std;


ostream &operator<<( ostream &output, const Move &m ) {
    
    output << "[Move #" << m.seq << " " << (int)m.move_type <<  " t=" <<m.t << " (" ;

    for(auto &xi : m.x){
        output << xi << ", ";
    }

    output << ")]" << endl; 

    return output;
}
