/**
 * Crazyflie safety filter controller interface.
 */
#ifndef __CONTROLLER_SAFETY_FILTER_H__
#define __CONTROLLER_SAFETY_FILTER_H__

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void controllerSafetyFilterInit(void);
bool controllerSafetyFilterTest(void);
void controllerSafetyFilter(control_t *control, const setpoint_t *setpoint,
                               const sensorData_t *sensors,
                               const state_t *state,
                               const stabilizerStep_t stabilizerStep);

#ifdef __cplusplus
}
#endif

#endif // __CONTROLLER_SAFETY_FILTER_H__
