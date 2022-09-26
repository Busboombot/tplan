#include "trj_types.h"

class IPacketSerial {

public:

    IPacketSerial() = default;

    virtual ~IPacketSerial() = default;

    virtual size_t available() = 0;

    virtual bool empty() = 0;

    virtual void pop() = 0;

    virtual vector<uint8_t> &front() = 0;

    virtual void send(const uint8_t* buffer, size_t size)  = 0;

    virtual void update() = 0;

};
