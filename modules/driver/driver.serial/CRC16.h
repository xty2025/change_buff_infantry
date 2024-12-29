// CICECOO 24.8.5

#include <cstdint>
#include "interfaceType.hpp"

namespace serial {
    void addCRC16(RawSerialWriteData *msg);
    //bool verifyCRC16(SerialReadData::RobotStatus *data);
    bool verifyCRC16(RawSerialData *data);
}