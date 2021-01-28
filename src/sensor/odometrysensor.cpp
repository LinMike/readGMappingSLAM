#include "sensor/odometrysensor.h"

namespace GMapping {

    OdometrySensor::OdometrySensor( const std::string& name, bool ideal ): Sensor(name) {
        m_ideal = ideal;
    }

} // end namespace