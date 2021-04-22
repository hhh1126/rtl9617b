/*
 * qtn_thermal.h
 *
 * The definitions for thermal mitigation support
 *
 * Copyright (c) 2020 Quantenna Communications, Inc.
 */

#ifndef __QTN_THERMAL_H__
#define __QTN_THERMAL_H__

/** @} */

/**
 * @addtogroup PowerAPIs
 * @{
 */

/** Minimum number of thermal mitigation stages supported. */
#define QTN_THERMAL_STAGE_MIN			1

/** Maximum number of thermal mitigation stages supported. */
#define QTN_THERMAL_STAGE_MAX			2

/** Default stage. ie. without thermal mitigation. */
#define QTN_THERMAL_STAGE_DEFAULT		0

/** Minimum value of temperature thersholds (in degree celsius). */
#define QTN_THERMAL_TEMP_DEGC_MIN		60

/** Maximum value of temperature thersholds (in degree celsius). */
#define QTN_THERMAL_TEMP_DEGC_MAX		120

/** Minimum value of radio id. */
#define QTN_THERMAL_RADIO_ID_MIN		0

/** Maximum value of radio id. */
#define QTN_THERMAL_RADIO_ID_MAX		2

/** Minimum number of transmit chains. */
#define QTN_THERMAL_TX_CHAINS_MIN		1

/** Maximum number of transmit chains. */
#define QTN_THERMAL_TX_CHAINS_MAX		8

/** Minimum value of transmit power backoff (in dB). */
#define QTN_THERMAL_TX_PWR_BACKOFF_DB_MIN	0

/** Maximum value of transmit power backoff (in dB). */
#define QTN_THERMAL_TX_PWR_BACKOFF_DB_MAX	100

/** Default value of transmit power backoff (in dB). */
#define QTN_THERMAL_TX_PWR_BACKOFF_DB_DEFAULT	0

/** Minimum value of polling interval (in seconds). */
#define QTN_THERMAL_POLL_INTRVL_SEC_MIN		10

/** Maximum value of polling interval (in seconds). */
#define QTN_THERMAL_POLL_INTRVL_SEC_MAX		600

/** Default value of polling interval (in seconds). */
#define QTN_THERMAL_POLL_INTRVL_SEC_DEFAULT	30

/**
 * Thermal mitigation message types used for configuration and retrieve status
 */
enum qtn_thermal_msg_type {
	QTN_THERMAL_MSG_TYPE_INIT = 1,
	QTN_THERMAL_MSG_TYPE_INTERVAL,
	QTN_THERMAL_MSG_TYPE_RADIO_CONFIG,
	QTN_THERMAL_MSG_TYPE_START,
	QTN_THERMAL_MSG_TYPE_STOP,
	QTN_THERMAL_MSG_TYPE_EXIT,
	QTN_THERMAL_MSG_TYPE_GET_STATUS,
	QTN_THERMAL_MSG_TYPE_OPER_STATUS,
	QTN_THERMAL_MSG_TYPE_MAX
};

/**
 * Return code in response to the thermal mitigation messages
 */
enum qtn_thermal_ret_code {
	QTN_THERMAL_RET_CODE_SUCCESS = 1,
	QTN_THERMAL_RET_CODE_INVALID_COMMAND,
	QTN_THERMAL_RET_CODE_INVALID_FLAGS,
	QTN_THERMAL_RET_CODE_INVALID_RECORDS,
	QTN_THERMAL_RET_CODE_INVALID_INTRVL,
	QTN_THERMAL_RET_CODE_INVALID_STAGE,
	QTN_THERMAL_RET_CODE_INVALID_TEMPERATURE,
	QTN_THERMAL_RET_CODE_INVALID_RADIO_ID,
	QTN_THERMAL_RET_CODE_INVALID_TX_CHAINS,
	QTN_THERMAL_RET_CODE_INVALID_TX_CHAINS_ON_RADIO,
	QTN_THERMAL_RET_CODE_INVALID_TX_PWR_BACKOFF,
	QTN_THERMAL_RET_CODE_INVALID_STATUS,
	QTN_THERMAL_RET_CODE_RADIO_NOT_ENABLED,
	QTN_THERMAL_RET_CODE_NO_TEMP_SENSOR,
	QTN_THERMAL_RET_CODE_ALREADY_ENABLED,
	QTN_THERMAL_RET_CODE_NSM_ENABLED,
	QTN_THERMAL_RET_CODE_STAGE_NOT_CONFIGURED,
	QTN_THERMAL_RET_CODE_OVERLAPPING_STAGE_TEMPS,
	QTN_THERMAL_RET_CODE_INTERNAL,
	QTN_THERMAL_RET_CODE_UNKNOWN,
	QTN_THERMAL_RET_CODE_MAX
};

/**
 * The data structure representing a configuration message used for thermal mitigation.
 * The generic arguments in this structure are used to represent various parameters
 * based on the message type \ref qtn_thermal_msg_type defined in the param 'msg_type'.
 */
struct qtn_thermal_cfg_param {
	/**
	 * Thermal mitigation message type \ref qtn_thermal_msg_type
	 */
	uint32_t msg_type;

	/**
	 * Radio Id \ref radio_id
	 */
	uint32_t radio_id;

	/**
	 * Generic argument 1. Usage based on the message type param 'msg_type'.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_INTERVAL, arg1 will represent
	 * polling interval.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_RADIO_CONFIG, arg1 will represent stage id.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_OPER_STATUS, arg1 will represent the thermal
	 * mitigation configuration status - 0 (not configured), (1 configured).
	 */
	uint32_t arg1;

	/**
	 * Generic argument 2. Usage based on the message type param 'msg_type'.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_RADIO_CONFIG, arg2 will represent
	 * low temperature threshold.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_OPER_STATUS, arg1 will represent stage id.
	 */
	uint32_t arg2;

	/**
	 * Generic argument 3. Usage based on the message type param 'msg_type'.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_RADIO_CONFIG, arg3 will represent
	 * low temperature threshold.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_OPER_STATUS, arg3 will represent
	 * radio temperature (integer part).
	 */
	uint32_t arg3;

	/**
	 * Generic argument 4. Usage based on the message type param 'msg_type'.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_RADIO_CONFIG, arg4 will represent tx chains.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_OPER_STATUS, arg4 will represent
	 * radio temperature (decimal part).
	 */
	uint32_t arg4;

	/**
	 * Generic argument 5. Usage based on the message type param 'msg_type'.
	 * * When msg_type is QTN_THERMAL_MSG_TYPE_RADIO_CONFIG, arg5 will represent
	 * tx power backoff.
	 */
	uint32_t arg5;

	/**
	 * Generic argument 6. Usage based on the message type param 'msg_type'.
	 */
	uint32_t arg6;

	/**
	 * Generic argument 7. Usage based on the message type param 'msg_type'.
	 */
	uint32_t arg7;

	/**
	 * Generic argument 8. Usage based on the message type param 'msg_type'.
	 */
	uint32_t arg8;
};

/**
 * Maximum number of records in the thermal mitigation configuration command.
 * See \ref qtn_thermal_cfg_records
 */
#define QTN_THERMAL_CMD_MAX_RECORDS	32

/**
 * Configuration command used used for thermal mitigation
 */
struct qtn_thermal_cfg_records {
	/**
	 * Flags for future use
	 * - Bits 0 - 31 are reserved and should be set to zero
	 */
	uint32_t	flags;
	/**
	 * Return code. \ref qtn_thermal_ret_code
	 */
	uint32_t	ret_code;
	/**
	 * Return status of thermal mitigation. 0 (disabled) or 1 (enabled)
	 */
	uint32_t	ret_status;
	/**
	 * Number of valid records in the param record[]
	 */
	uint32_t	num_records;
	/**
	 * Records of configuration params.  See \ref qtn_thermal_cfg_param
	 */
	struct qtn_thermal_cfg_param record[QTN_THERMAL_CMD_MAX_RECORDS];
};

/** @} */

#define QTN_THERMAL_CHECK_LIMITS(_val, _low, _high) \
		((((_val) >= (_low)) && ((_val) <= (_high))) ? 1 : 0)
#define QTN_THERMAL_STAGE_VALID(_val) \
		QTN_THERMAL_CHECK_LIMITS((_val), \
		QTN_THERMAL_STAGE_MIN, QTN_THERMAL_STAGE_MAX)
#define QTN_THERMAL_TEMP_VALID(_val) \
		QTN_THERMAL_CHECK_LIMITS((_val), \
		QTN_THERMAL_TEMP_DEGC_MIN, QTN_THERMAL_TEMP_DEGC_MAX)
#define QTN_THERMAL_RADIO_ID_VALID(_val) \
		(((_val) <= QTN_THERMAL_RADIO_ID_MAX) ? 1 : 0)
#define QTN_THERMAL_TX_CHAINS_VALID(_val) \
		QTN_THERMAL_CHECK_LIMITS((_val), \
		QTN_THERMAL_TX_CHAINS_MIN, QTN_THERMAL_TX_CHAINS_MAX)
#define QTN_THERMAL_TX_PWR_BACKOFF_VALID(_val) \
		(((_val) <= QTN_THERMAL_TX_PWR_BACKOFF_DB_MAX) ? 1 : 0)
#define QTN_THERMAL_POLL_INTRVL_VALID(_val) \
		QTN_THERMAL_CHECK_LIMITS((_val), \
		QTN_THERMAL_POLL_INTRVL_SEC_MIN, QTN_THERMAL_POLL_INTRVL_SEC_MAX)

#define QTN_THERMAL_MSG_TYPE_VALID(_cmd) \
		QTN_THERMAL_CHECK_LIMITS((_cmd), 1, (QTN_THERMAL_MSG_TYPE_MAX - 1))

#endif /* __QTN_THERMAL_H__ */

