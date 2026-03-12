#ifndef TOOL_DATA_HPP
#define TOOL_DATA_HPP

#ifdef _WIN32
#pragma warning( disable: 4251 )
#endif

#ifdef _WIN32
    #ifdef CAPICOMMON_EXPORTS
        #define CAPICOMMON_API __declspec(dllexport)
    #else
        #define CAPICOMMON_API __declspec(dllimport)
    #endif
#else
    #define CAPICOMMON_API __attribute__ ((visibility ("default")))
#endif

#include <string>
#include <vector>
#include <stdint.h> 

#include "MarkerData.h"
#include "Transform.h"
#include "SystemAlert.h"

class CAPICOMMON_API ToolData
{
public:
    ToolData();
    virtual ~ToolData(){};

    static const int PRECISION = 6;

    uint32_t frameNumber;

    Transform transform;

    uint16_t systemStatus;

    uint32_t portStatus;

    uint8_t frameType;

    uint8_t frameSequenceIndex;

    uint16_t frameStatus;

    uint32_t timespec_s;

    uint32_t timespec_ns;

    std::vector<MarkerData> markers;

    std::vector<uint8_t> buttons;

    std::vector<SystemAlert> systemAlerts;

    bool dataIsNew;

    std::string toolInfo;
};

namespace SystemStatus
{
    /* bit 0 */ static const int CommSyncError = 0x0001;
    /* bit 3 */ static const int ProcessingException = 0x0008;
    /* bit 6 */ static const int PortOccupied = 0x0040;
    /* bit 7 */ static const int PortUnoccupied = 0x0080;
    /* bit 8 */ static const int DiagnosticPending = 0x0100;
    /* bit 9 */ static const int TemperatureOutOfRange = 0x0200;
    /* bit 10 */ static const int HardwareConfigChanged = 0x0400;

    CAPICOMMON_API std::string toString(uint16_t systemStatus);
}

namespace FrameType
{
    enum CAPICOMMON_API value { Dummy = 0x00,
                                ActiveWireless = 0x01,
                                Passive = 0x02,
                                Active = 0x03,
                                Laser = 0x04,
                                Illuminated = 0x05,
                                Background = 0x06,
                                Magnetic = 0x07 };

    CAPICOMMON_API std::string toString(uint8_t frameType);
}

namespace ButtonState
{
    enum CAPICOMMON_API value {Open = 0x00, Closed = 0x01};

    CAPICOMMON_API std::string toString(uint8_t state);
}

#endif // TOOL_DATA_HPP
