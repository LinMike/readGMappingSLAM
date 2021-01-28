#include "sensor/sensorreading.h"

namespace GMapping {

    SensorReading::SensorReading( const Sensor* s, double time ) {
        m_sensor = s;
        m_time = time;
    }

    SensorReading::~SensorReading() {
    }

} // end namespace