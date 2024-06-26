#ifndef __SRC_HALMET_DIGITAL_H__
#define __SRC_HALMET_DIGITAL_H__

#include "sensesp/sensors/sensor.h"

using namespace sensesp;

FloatProducer* ConnectTachoSender(int pin, String path_prefix, String sk_name,
                                  int sort_order_base);
BoolProducer* ConnectAlarmSender(int pin, String name, int sort_order_base);

#endif
