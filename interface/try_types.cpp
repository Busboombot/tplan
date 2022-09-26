#include "trj_types.h"
#include <iostream>

ostream &operator<<( ostream &output, const Message &m ){

    output << "[M " << m.header.seq << " " << (int)m.header.code << " ";

    switch(m.header.code){
        case CommandCode::MESSAGE:
            cout << "Message " << string(m.buffer.data()) ;
            break;
        case CommandCode::ECHO:
            cout << "Echo " << string(m.buffer.data()) ;
            break;

    }

    cout << "]";
    return output;

}