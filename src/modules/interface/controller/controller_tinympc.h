/**
 * Crazyflie TinyMPC controller interface.
 */
#ifndef __CONTROLLER_TINYMPC_H__
#define __CONTROLLER_TINYMPC_H__

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void controllerTinyMPCFirmwareInit(void);
bool controllerTinyMPCFirmwareTest(void);
void controllerTinyMPCFirmware(control_t *control, const setpoint_t *setpoint,
                               const sensorData_t *sensors,
                               const state_t *state,
                               const stabilizerStep_t stabilizerStep);

#ifdef __cplusplus
}
#endif

#endif // __CONTROLLER_TINYMPC_H__
