/**
 *  @file
 *
 *  @brief Configuration header for the AppController.c file.
 *
 */

/* header definition ******************************************************** */
#ifndef APPCONTROLLER_H_
#define APPCONTROLLER_H_

/* local interface declaration ********************************************** */
#include "XDK_Utils.h"

/* local type and macro definitions */

/* local module global variable declarations */

/* local inline function definitions */

/* Macro to convert readable representation of IPv4 in terms of uint32_t variable */
/**
 *  * APP_CONTROLLER_TX_DELAY is sensor data transmission in milliseconds
 */
#define APP_CONTROLLER_TX_DELAY             UINT32_C(20)

/**
 * APP_CURRENT_RATED_TRANSFORMATION_RATIO is the current rated transformation ratio.
 * Unused if APP_SENSOR_CURRENT is false.
 * This will vary from one external LEM sensor to another.
 */
#define APP_CURRENT_RATED_TRANSFORMATION_RATIO      (0)
/**
 * @brief Gives control to the Application controller.
 *
 * @param[in] cmdProcessorHandle
 * Handle of the main command processor which shall be used based on the application needs
 *
 * @param[in] param2
 * Unused
 */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2);

#endif /* APPCONTROLLER_H_ */
