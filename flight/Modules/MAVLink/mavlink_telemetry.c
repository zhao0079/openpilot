/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{ 
 * @addtogroup TelemetryModule Telemetry Module
 * @brief MAVLink telemetry module
 * Starts three tasks (RX, TX, and priority TX) that watch event queues
 * and handle all the telemetry of the UAVobjects
 * @{ 
 *
 * @file       mavlink_telemetry.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      MAVLink Telemetry module, handles telemetry and UAVObject updates
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "openpilot.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"
#include "manualcontrolcommand.h"
#include "telemetrysettings.h"

// Private constants
#define MAX_QUEUE_SIZE   TELEM_QUEUE_SIZE
#define STACK_SIZE_BYTES PIOS_TELEM_STACK_SIZE
#define TASK_PRIORITY_RX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TXPRI (tskIDLE_PRIORITY + 2)
#define REQ_TIMEOUT_MS 250
#define MAX_RETRIES 2
#define STATS_UPDATE_PERIOD_MS 4000
#define CONNECTION_TIMEOUT_MS 8000

// Private types

// Private variables
static uint32_t telemetryPort;
static xQueueHandle queue;

#if defined(PIOS_TELEM_PRIORITY_QUEUE)
static xQueueHandle priorityQueue;
static xTaskHandle telemetryTxPriTaskHandle;
static void telemetryTxPriTask(void *parameters);
#else
#define priorityQueue queue
#endif

static xTaskHandle telemetryTxTaskHandle;
static xTaskHandle telemetryRxTaskHandle;
static uint32_t txErrors;
static uint32_t txRetries;
static TelemetrySettingsData settings;
static uint32_t timeOfLastObjectUpdate;

// Private functions
static void telemetryTxTask(void *parameters);
static void telemetryRxTask(void *parameters);
static int32_t transmitData(uint8_t * data, int32_t length);
static void registerObject(UAVObjHandle obj);
static void updateObject(UAVObjHandle obj);
static int32_t addObject(UAVObjHandle obj);
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs);
static void processObjEvent(UAVObjEvent * ev);
static void updateTelemetryStats();
static void gcsTelemetryStatsUpdated();
static void updateSettings();


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
static mavlink_system_t mavlink_system;
static mavlink_message_t rx_msg;
static mavlink_message_t tx_msg;
static mavlink_status_t rx_status;
static uint8_t mavlinkTxBuf[MAVLINK_MAX_PACKET_LEN];

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
    	uint32_t outputPort;

    	// Determine input port (USB takes priority over telemetry port)
    #if defined(PIOS_INCLUDE_USB_HID)
    	if (PIOS_USB_HID_CheckAvailable(0)) {
    		outputPort = PIOS_COM_TELEM_USB;
    	} else
    #endif /* PIOS_INCLUDE_USB_HID */
    	{
    		outputPort = telemetryPort;
    	}

    	PIOS_COM_SendBufferNonBlocking(outputPort, &ch, 1);
    }
    if (chan == MAVLINK_COMM_1)
    {
    	PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_USB, &ch, 1);
    }
}

#include "common/mavlink.h"

/**
 * Initialise the telemetry module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t MAVLinkInitialize(void)
{
	UAVObjEvent ev;

	// Initialize vars
	timeOfLastObjectUpdate = 0;

	// Create object queues
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	priorityQueue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#endif
	
	// Get telemetry settings object
	updateSettings();

	// Initialise UAVTalk
	UAVTalkInitialize(&transmitData);

	// Initialize MAVLink
	mavlink_system.sysid = 100; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

	mavlink_parse_state_initialize(&rx_status);

	// Process all registered objects and connect queue for updates
	UAVObjIterate(&registerObject);

	// Create periodic event that will be used to update the telemetry stats
	txErrors = 0;
	txRetries = 0;
	memset(&ev, 0, sizeof(UAVObjEvent));
	EventPeriodicQueueCreate(&ev, priorityQueue, STATS_UPDATE_PERIOD_MS);

	// Listen to objects of interest
	GCSTelemetryStatsConnectQueue(priorityQueue);
	TelemetrySettingsConnectQueue(priorityQueue);

	// Start telemetry tasks
	xTaskCreate(telemetryTxTask, (signed char *)"MAVLTx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_TX, &telemetryTxTaskHandle);
	xTaskCreate(telemetryRxTask, (signed char *)"MAVLRx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_RX, &telemetryRxTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYTX, telemetryTxTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYRX, telemetryRxTaskHandle);

#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	xTaskCreate(telemetryTxPriTask, (signed char *)"MAVLPriTx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_TXPRI, &telemetryTxPriTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYTXPRI, telemetryTxPriTaskHandle);
#endif
	
	return 0;
}

/**
 * Register a new object, adds object to local list and connects the queue depending on the object's
 * telemetry settings.
 * \param[in] obj Object to connect
 */
static void registerObject(UAVObjHandle obj)
{
	// Setup object for periodic updates
	addObject(obj);

	// Setup object for telemetry updates
	updateObject(obj);
}

/**
 * Update object's queue connections and timer, depending on object's settings
 * \param[in] obj Object to updates
 */
static void updateObject(UAVObjHandle obj)
{
	UAVObjMetadata metadata;
	int32_t eventMask;

	// Get metadata
	UAVObjGetMetadata(obj, &metadata);

	// Setup object depending on update mode
	if (metadata.telemetryUpdateMode == UPDATEMODE_PERIODIC) {
		// Set update period
		setUpdatePeriod(obj, metadata.telemetryUpdatePeriod);
		// Connect queue
		eventMask = EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_ONCHANGE) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Connect queue
		eventMask = EV_UPDATED | EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_MANUAL) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Connect queue
		eventMask = EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_NEVER) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Disconnect queue
		UAVObjDisconnectQueue(obj, priorityQueue);
	}
}

// FIXME XXX
#include "attitudeactual.h"
#include "attituderaw.h"
#include "systemstats.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"

FlightTelemetryStatsData flightStats;
GCSTelemetryStatsData gcsTelemetryStatsData;
static AttitudeActualData attitudeActual;
static AttitudeRawData attitudeRaw;
static ManualControlCommandData manualControl;
static mavlink_raw_imu_t attitude_raw;
static mavlink_attitude_t attitude;
static mavlink_rc_channels_raw_t rc_channels;
static mavlink_debug_vect_t debug;


/**
 * Processes queue events
 */
static void processObjEvent(UAVObjEvent * ev)
{
//	UAVObjMetadata metadata;
//	FlightTelemetryStatsData flightStats;
//	GCSTelemetryStatsData gcsTelemetryStatsData;
//	int32_t retries;
//	int32_t success;

	if (ev->obj == 0) {
		updateTelemetryStats();
	} else if (ev->obj == GCSTelemetryStatsHandle()) {
		gcsTelemetryStatsUpdated();
	} else if (ev->obj == TelemetrySettingsHandle()) {
		updateSettings();
	} else {
		mavlink_message_t msg;

		mavlink_system.sysid = 20;
		mavlink_system.compid = MAV_COMP_ID_IMU;
		mavlink_system.type = MAV_FIXED_WING;
		uint8_t mavClass = MAV_CLASS_OPENPILOT;


		uint32_t objId;

		// Setup type and object id fields
		objId = UAVObjGetID(ev->obj);

//		uint64_t timeStamp = 0;
		switch(objId) {
			case ATTITUDEACTUAL_OBJID:
			{
				AttitudeActualGet(&attitudeActual);
				AttitudeRawGet(&attitudeRaw);

				// Copy data
				attitude_raw.xacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_X];
				attitude_raw.yacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_Y];
				attitude_raw.zacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_Z];
				attitude_raw.xgyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_X];
				attitude_raw.ygyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_Y];
				attitude_raw.zgyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_Z];
				attitude_raw.xmag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_X];
				attitude_raw.ymag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_Y];
				attitude_raw.zmag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_Z];

				mavlink_msg_raw_imu_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &attitude_raw);
				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);

				attitude.roll  = (attitudeActual.Roll/180.0f)*3.14159265f;
				attitude.pitch = (attitudeActual.Pitch/180.0f)*3.14159265f;
				attitude.yaw   = (attitudeActual.Yaw/180.0f)*3.14159265f;

				attitude.rollspeed  = (attitudeActual.RollSpeed/180.0f)*3.14159265f;
				attitude.pitchspeed = (attitudeActual.PitchSpeed/180.0f)*3.14159265f;
				attitude.yawspeed   = (attitudeActual.YawSpeed/180.0f)*3.14159265f;

				mavlink_msg_attitude_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &attitude);
				// Copy the message to the send buffer
				len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
//
//				mavlink_msg_attitude_send(MAVLINK_COMM_0, timeStamp,attitudeActual.Roll,
//						attitudeActual.Pitch,attitudeActual.Yaw,
//						attitudeRaw.gyros[ATTITUDERAW_GYROS_X],
//						attitudeRaw.gyros[ATTITUDERAW_GYROS_Y],
//						attitudeRaw.gyros[ATTITUDERAW_GYROS_Z]);
				break;
			}
			case FLIGHTTELEMETRYSTATS_OBJID:
			{
//				FlightTelemetryStatsData flightTelemetryStats;
				FlightTelemetryStatsGet(&flightStats);

				// XXX this is a hack to make it think it got a confirmed
				// connection
//				flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
//				GCSTelemetryStatsGet(&gcsTelemetryStatsData);
//				gcsTelemetryStatsData.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
//
//
//				//mavlink_msg_heartbeat_send(MAVLINK_COMM_0,mavlink_system.type,mavClass);
//				mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, mavClass);
//				// Copy the message to the send buffer
//				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
//				// Send buffer
//				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
				break;
			}
			case SYSTEMSTATS_OBJID:
			{
				//mavlink_msg_heartbeat_send(MAVLINK_COMM_0,mavlink_system.type,mavClass);
				mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, mavClass);
				//mavlink_msg_cpu_load_pack(mavlink_system.sysid, mavlink_system.compid, &msg,ucCpuLoad,ucCpuLoad,0);
				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);

//				uint8_t ucCpuLoad;
//				SystemStatsCPULoadGet(&ucCpuLoad);
//				mavlink_msg_debug_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, (float)ucCpuLoad);
//				len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
//				// Send buffer
//				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
				break;
			}
			case MANUALCONTROLCOMMAND_OBJID:
			{
				manualControl.Channel[0] = PIOS_PPM_Get(0);
				manualControl.Channel[1] = PIOS_PPM_Get(1);

				rc_channels.chan1_raw = manualControl.Channel[0];
				rc_channels.chan2_raw = manualControl.Channel[1];

				debug.x = PIOS_PPM_Get(0);
				debug.y = PIOS_PPM_Get(1);
				debug.z = PIOS_PPM_Get(2);
				debug.name[0] = 'R';
				debug.name[1] = 'C';
				debug.name[2] = 0;
				debug.usec = 0;

				mavlink_msg_debug_vect_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &debug);



//				mavlink_msg_rc_channels_raw_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &rc_channels);


				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
				break;
			}
			default:
			{
				//printf("unknown object: %x\n",(unsigned int)objId);
				break;
			}
		}







//		if (ev->obj == AttitudeRawHandle()) {
//			// Get object data
//			mavlink_raw_imu_t imu;
//			imu.xacc =
//		}

		// Send buffer
		//transmitData(buf, len);
		//if (outStream!=NULL) (*outStream)(txBuffer, dataOffset+length+CHECKSUM_LENGTH);

//		// Only process event if connected to GCS or if object FlightTelemetryStats is updated
//		FlightTelemetryStatsGet(&flightStats);
//		if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED || ev->obj == FlightTelemetryStatsHandle()) {
//			// Get object metadata
//			UAVObjGetMetadata(ev->obj, &metadata);
//			// Act on event
//			retries = 0;
//			success = -1;
//			if (ev->event == EV_UPDATED || ev->event == EV_UPDATED_MANUAL) {
//				// Send update to GCS (with retries)
//				while (retries < MAX_RETRIES && success == -1) {
//					success = UAVTalkSendObject(ev->obj, ev->instId, metadata.telemetryAcked, REQ_TIMEOUT_MS);	// call blocks until ack is received or timeout
//					++retries;
//				}
//				// Update stats
//				txRetries += (retries - 1);
//				if (success == -1) {
//					++txErrors;
//				}
//			} else if (ev->event == EV_UPDATE_REQ) {
//				// Request object update from GCS (with retries)
//				while (retries < MAX_RETRIES && success == -1) {
//					success = UAVTalkSendObjectRequest(ev->obj, ev->instId, REQ_TIMEOUT_MS);	// call blocks until update is received or timeout
//					++retries;
//				}
//				// Update stats
//				txRetries += (retries - 1);
//				if (success == -1) {
//					++txErrors;
//				}
//			}
//			// If this is a metaobject then make necessary telemetry updates
//			if (UAVObjIsMetaobject(ev->obj)) {
//				updateObject(UAVObjGetLinkedObj(ev->obj));	// linked object will be the actual object the metadata are for
//			}
//		}
	}
}

/**
 * Telemetry transmit task, regular priority
 */
static void telemetryTxTask(void *parameters)
{
	UAVObjEvent ev;

	// Loop forever
	while (1) {
		// Wait for queue message
		if (xQueueReceive(queue, &ev, portMAX_DELAY) == pdTRUE) {
			// Process event
			processObjEvent(&ev);
		}
	}
}

/**
 * Telemetry transmit task, high priority
 */
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
static void telemetryTxPriTask(void *parameters)
{
	UAVObjEvent ev;

	// Loop forever
	while (1) {
		// Wait for queue message
		if (xQueueReceive(priorityQueue, &ev, portMAX_DELAY) == pdTRUE) {
			// Process event
			processObjEvent(&ev);
		}
	}
}
#endif


//static uint32_t m_parameter_i = 0;

void execute_action(uint8_t action)
{
	switch (action)
	{
	case MAV_ACTION_LAUNCH:
//		if (global_data.state.mav_mode > (uint8_t)MAV_MODE_LOCKED)
//		{
//			global_data.state.status = (uint8_t)MAV_STATE_ACTIVE;
//		}
		break;
	case MAV_ACTION_MOTORS_START:
//		if (global_data.state.mav_mode > (uint8_t)MAV_MODE_LOCKED)
//		{
//			global_data.state.status = (uint8_t)MAV_STATE_ACTIVE;
//		}
		break;
	case MAV_ACTION_MOTORS_STOP:
//		global_data.state.status = (uint8_t)MAV_STATE_STANDBY;
		break;
	case MAV_ACTION_EMCY_KILL:
//		global_data.state.status = (uint8_t)MAV_STATE_EMERGENCY;
		break;
	case MAV_ACTION_STORAGE_READ:
//		param_read_all();
//		debug_message_buffer("Started reading params from eeprom");
		break;
	case MAV_ACTION_STORAGE_WRITE:
//		debug_message_buffer("Started writing params to eeprom");
//		param_write_all();
		break;
	case MAV_ACTION_CALIBRATE_GYRO:
//		start_gyro_calibration();
		break;
	case MAV_ACTION_CALIBRATE_RC:
//		start_rc_calibration();
		break;
	case MAV_ACTION_CALIBRATE_MAG:
//		start_mag_calibration();
		break;
	case MAV_ACTION_CALIBRATE_PRESSURE:
//		start_pressure_calibration();
		break;
	case MAV_ACTION_SET_ORIGIN:
		// If not flying
//		if (!sys_state_is_flying())
//		{
//			gps_set_local_origin();
//			altitude_set_local_origin();
//		}
		break;
	default:
		// Should never be reached, ignore unknown commands
//		debug_message_buffer_sprintf("Rejected unknown action Number: %u", action);
		break;
	}
}


/**
 * Telemetry transmit task. Processes queue events and periodic updates.
 */
static void telemetryRxTask(void *parameters)
{
	uint32_t inputPort;
	int32_t len;

	// Task loop
	while (1) {
#if defined(PIOS_INCLUDE_USB_HID)
		// Determine input port (USB takes priority over telemetry port)
		if (PIOS_USB_HID_CheckAvailable(0)) {
			inputPort = PIOS_COM_TELEM_USB;
		} else
#endif /* PIOS_INCLUDE_USB_HID */
		{
			inputPort = telemetryPort;
		}

		// Block until data are available
		// TODO: Currently we periodically check the buffer for data, update once the PIOS_COM is made blocking
		len = PIOS_COM_ReceiveBufferUsed(inputPort);
		for (int32_t n = 0; n < len; ++n) {
			if (mavlink_parse_char(MAVLINK_COMM_0, PIOS_COM_ReceiveBuffer(inputPort), &rx_msg, &rx_status))
			{

				switch (rx_msg.msgid)
				{
				case MAVLINK_MSG_ID_SET_MODE:
				{
					mavlink_set_mode_t mode;
					mavlink_msg_set_mode_decode(&rx_msg, &mode);
					// Check if this system should change the mode
					if (mode.target == mavlink_system.sysid)
					{
						//sys_set_mode(mode.mode);

						uint8_t mav_mode = MAV_MODE_LOCKED;
						uint8_t nav_mode = MAV_NAV_LOST;
						mav_mode = mode.mode;
						uint8_t mav_state = MAV_STATE_ACTIVE;
						uint16_t vbat = 11000;

						// Emit current mode
						mavlink_msg_sys_status_pack_chan(mavlink_system.sysid, mavlink_system.compid, MAVLINK_COMM_0, &tx_msg, mav_mode, nav_mode,
								mav_state, 0,vbat, 0, 0);
						// Send message
						uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &tx_msg);
						// Send buffer
						PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);

					}
				}
				break;
//				case MAVLINK_MSG_ID_ACTION:
//				{
//					execute_action(mavlink_msg_action_get_action(msg));
//
//					//Forwart actions from Xbee to Onboard Computer and vice versa
//					if (chan == MAVLINK_COMM_1)
//					{
//						mavlink_send_uart(MAVLINK_COMM_0, msg);
//					}
//					else if (chan == MAVLINK_COMM_0)
//					{
//						mavlink_send_uart(MAVLINK_COMM_1, msg);
//					}
//				}
//				break;
			}

		}
		}
		vTaskDelay(5);	// <- remove when blocking calls are implemented

	}
}

/**
 * Transmit data buffer to the modem or USB port.
 * \param[in] data Data buffer to send
 * \param[in] length Length of buffer
 * \return 0 Success
 */
static int32_t transmitData(uint8_t * data, int32_t length)
{
	uint32_t outputPort;

	// Determine input port (USB takes priority over telemetry port)
#if defined(PIOS_INCLUDE_USB_HID)
	if (PIOS_USB_HID_CheckAvailable(0)) {
		outputPort = PIOS_COM_TELEM_USB;
	} else
#endif /* PIOS_INCLUDE_USB_HID */
	{
		outputPort = telemetryPort;
	}

	return PIOS_COM_SendBufferNonBlocking(outputPort, data, length);
}

/**
 * Setup object for periodic updates.
 * \param[in] obj The object to update
 * \return 0 Success
 * \return -1 Failure
 */
static int32_t addObject(UAVObjHandle obj)
{
	UAVObjEvent ev;

	// Add object for periodic updates
	ev.obj = obj;
	ev.instId = UAVOBJ_ALL_INSTANCES;
	ev.event = EV_UPDATED_MANUAL;
	return EventPeriodicQueueCreate(&ev, queue, 0);
}

/**
 * Set update period of object (it must be already setup for periodic updates)
 * \param[in] obj The object to update
 * \param[in] updatePeriodMs The update period in ms, if zero then periodic updates are disabled
 * \return 0 Success
 * \return -1 Failure
 */
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs)
{
	UAVObjEvent ev;

	// Add object for periodic updates
	ev.obj = obj;
	ev.instId = UAVOBJ_ALL_INSTANCES;
	ev.event = EV_UPDATED_MANUAL;
	return EventPeriodicQueueUpdate(&ev, queue, updatePeriodMs);
}

/**
 * Called each time the GCS telemetry stats object is updated.
 * Trigger a flight telemetry stats update if a connection is not
 * yet established.
 */
static void gcsTelemetryStatsUpdated()
{
	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);
	if (flightStats.Status != FLIGHTTELEMETRYSTATS_STATUS_CONNECTED || gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED) {
		updateTelemetryStats();
	}
}

/**
 * Update telemetry statistics and handle connection handshake
 */
static void updateTelemetryStats()
{
	UAVTalkStats utalkStats;
	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	uint8_t forceUpdate;
	uint8_t connectionTimeout;
	uint32_t timeNow;

	// Get stats
	UAVTalkGetStats(&utalkStats);
	UAVTalkResetStats();

	// Get object data
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);

	// Update stats object
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		flightStats.RxDataRate = (float)utalkStats.rxBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		flightStats.TxDataRate = (float)utalkStats.txBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		flightStats.RxFailures += utalkStats.rxErrors;
		flightStats.TxFailures += txErrors;
		flightStats.TxRetries += txRetries;
		txErrors = 0;
		txRetries = 0;
	} else {
		flightStats.RxDataRate = 0;
		flightStats.TxDataRate = 0;
		flightStats.RxFailures = 0;
		flightStats.TxFailures = 0;
		flightStats.TxRetries = 0;
		txErrors = 0;
		txRetries = 0;
	}

	// Check for connection timeout
	timeNow = xTaskGetTickCount() * portTICK_RATE_MS;
	if (utalkStats.rxObjects > 0) {
		timeOfLastObjectUpdate = timeNow;
	}
	if ((timeNow - timeOfLastObjectUpdate) > CONNECTION_TIMEOUT_MS) {
		connectionTimeout = 1;
	} else {
		connectionTimeout = 0;
	}

	// Update connection state
	forceUpdate = 1;
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED) {
		// Wait for connection request
		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK;
		}
	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK) {
		// Wait for connection
		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_CONNECTED) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
		} else if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_DISCONNECTED) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
		}
	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		if (gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED || connectionTimeout) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
		} else {
			forceUpdate = 0;
		}
	} else {
		flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	}

	// Update the telemetry alarm
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		AlarmsClear(SYSTEMALARMS_ALARM_TELEMETRY);
	} else {
		AlarmsSet(SYSTEMALARMS_ALARM_TELEMETRY, SYSTEMALARMS_ALARM_ERROR);
	}

	// Update object
	FlightTelemetryStatsSet(&flightStats);

	// Force telemetry update if not connected
	if (forceUpdate) {
		FlightTelemetryStatsUpdated();
	}
}

/**
 * Update the telemetry settings, called on startup and
 * each time the settings object is updated
 */
static void updateSettings()
{
    // Set port
    telemetryPort = PIOS_COM_TELEM_RF;

    // Retrieve settings
    TelemetrySettingsGet(&settings);

    // Set port speed
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_2400) PIOS_COM_ChangeBaud(telemetryPort, 2400);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_4800) PIOS_COM_ChangeBaud(telemetryPort, 4800);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_9600) PIOS_COM_ChangeBaud(telemetryPort, 9600);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_19200) PIOS_COM_ChangeBaud(telemetryPort, 19200);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_38400) PIOS_COM_ChangeBaud(telemetryPort, 38400);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_57600) PIOS_COM_ChangeBaud(telemetryPort, 57600);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_115200) PIOS_COM_ChangeBaud(telemetryPort, 115200);
}

/**
  * @}
  * @}
  */
