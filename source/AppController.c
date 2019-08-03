/**
 * @ingroup APPS_LIST
 *
 * @defgroup XDK110_ROS_SERIAL Xdk110RosSerial
 * @{
 *
 * @brief Application of printing all the defined sensors on serialport
 *
 * @details
 *
 * @file
 **/

/* module includes ********************************************************** */

/* own header files */
#include "XdkAppInfo.h"

#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_CONTROLLER

/* own header files */
#include "AppController.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "XDK_Sensor.h"
#include "XDK_Utils.h"
#include "BCDS_Assert.h"
#include "BCDS_CmdProcessor.h"
#include "XdkSensorHandle.h"
#include "XDK_VirtualSensor.h"
#include "FreeRTOS.h"
#include "task.h"

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */

static CmdProcessor_T *AppCmdProcessor;/**< Handle to store the main Command processor handle to be reused by ServalPAL thread */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */

static Sensor_Setup_T SensorSetup =
        {
                .CmdProcessorHandle = NULL,
                .Enable =
                        {
                                .Accel = true,
                                .Mag = true,
                                .Gyro = true,
                                .Humidity = true,
                                .Temp = true,
                                .Pressure = true,
                                .Light = true,
                                .Noise = false,
                        },
                .Config =
                        {
                                .Accel =
                                        {
                                                .Type = SENSOR_ACCEL_BMA280,
                                                .IsInteruptEnabled = false,
                                        },
                                .Gyro =
                                        {
                                                .Type = SENSOR_GYRO_BMG160,
                                                .IsRawData = false,
                                        },
                                .Mag =
                                        {
                                                .IsRawData = false,
                                        },
                                .Light =
                                        {
                                                .IsInteruptEnabled = false,
                                        }
                        },
        };/**< Sensor setup parameters */

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief This function controls the application flow
 * - Triggers Sensor data sampling
 * - Read the sampled Sensor data
 *
 * @param[in] pvParameters
 * Unused
 */
static void AppControllerFire(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);

    Retcode_T retcode = RETCODE_OK;
    Orientation_QuaternionData_T QuaternionValues = { INT32_C(0), INT32_C(0),
    		INT32_C(0), INT32_C(0) };
    LinearAcceleration_XyzMps2Data_T LinearAccValues = { INT32_C(0),
    	    INT32_C(0), INT32_C(0) };
    CalibratedGyro_XyzRpsData_T GyroscopeValues = { INT32_C(0),
    	    INT32_C(0), INT32_C(0) };

    while (1)
    {
        retcode = Orientation_readQuaternionValue(&QuaternionValues);
        if (RETCODE_OK == retcode) {
			printf("%f %f %f %f ",
					QuaternionValues.w, QuaternionValues.x, QuaternionValues.y, QuaternionValues.z);
        }

        retcode = LinearAcceleration_readXyzMps2Value(&LinearAccValues);
        if (RETCODE_OK == retcode)
        {
        	printf("%f %f %f ",
        			LinearAccValues.xAxisData, LinearAccValues.yAxisData, LinearAccValues.zAxisData);
        }

    	retcode = CalibratedGyro_readXyzRpsValue(&GyroscopeValues);
        if (RETCODE_OK == retcode)
        {
        	printf("%f %f %f\n",
        			GyroscopeValues.xAxisData, GyroscopeValues.yAxisData, GyroscopeValues.zAxisData);
        }

        if (RETCODE_OK != retcode)
        {
            Retcode_RaiseError(retcode);
        }
        vTaskDelay(pdMS_TO_TICKS(APP_CONTROLLER_TX_DELAY));
    }
}

/**
 * @brief To enable the necessary modules for the application
 * - Sensor
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerEnable(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    Retcode_T retcode = Sensor_Enable();
    if (RETCODE_OK == retcode)
    {
        if (pdPASS != xTaskCreate(AppControllerFire, (const char * const ) "AppController", TASK_STACK_SIZE_APP_CONTROLLER, NULL, TASK_PRIO_APP_CONTROLLER, &AppControllerHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);

        }
    }
    if (RETCODE_OK == retcode)
    {
    	retcode = Orientation_init(xdkOrientationSensor_Handle);
    	if ( RETCODE_OK != retcode) {
    		printf("Failed to initialize the OrientationSensor_Handle");
    		retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
    	}
    }
    if (RETCODE_OK == retcode)
	{
		retcode = LinearAcceleration_init(xdkLinearAccelSensor_Handle);
		if ( RETCODE_OK != retcode) {
			printf("Failed to initialize the LinearAccelSensor_Handle");
			retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
		}
	}
    if (RETCODE_OK == retcode)
    {
    	retcode = CalibratedGyro_init(XdkGyroscope_ClbrSensor_Handle);
    	if ( RETCODE_OK != retcode) {
			printf("Failed to initialize the Gyroscope_ClbrSensor_Handle");
			retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
		}
    }

    if (RETCODE_OK != retcode)
    {
        printf("AppControllerEnable : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
    Utils_PrintResetCause();
}

/**
 * @brief To setup the necessary modules for the application
 * - Sensor
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerSetup(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    SensorSetup.CmdProcessorHandle = AppCmdProcessor;
    Retcode_T retcode = Sensor_Setup(&SensorSetup);
    if (RETCODE_OK == retcode)
    {
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerEnable, NULL, UINT32_C(0));
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerSetup : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/* global functions ********************************************************** */

/** Refer interface header for description */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(param2);

    Retcode_T retcode = RETCODE_OK;

    if (cmdProcessorHandle == NULL)
    {
        printf("AppController_Init : Command processor handle is NULL \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerSetup, NULL, UINT32_C(0));
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

