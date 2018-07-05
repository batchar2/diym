package options

const (
	EVENT_UNDEFINED 	= -1 	// неизвестно
	EVENT_WAIT_CALL 	= 2		// ожидаем вызов
	EVENT_CALL 			= 1		// пошел вызов
	EVENT_NOTIF 		= 3		// польззователь получил вызов
	EVENT_USER_ACTION	= 4		// ждем действие от пользователя
	EVENT_UP_PHONE 		= 5
	EVENT_DOWN_PHONE 	= 6
	EVENT_OPEN_DOORE 	= 7
)
const (
	MQTT_CONTROL_SUB = "/dm/state/control"
	MQTT_DEVICE_PUB = "/dm/state/device"
)
const (
	MQTT_DEVICE_WAIT_CALL_NOTIF_STATE 	= "wait_call"
	MQTT_DEVICE_ACTION_CALL_STATE 		= "action_call"
	MQTT_USER_WAIT_NOTIF_STATE 			= "wait_user_notif"
	MQTT_DEVICE_ACTION_USER_ACTION		= "action_user_mancontrol"
	MQTT_USER_ACTION_UP_HONE_STATE 		= "action_up_phone"
	MQTT_USER_ACTION_DOWN_PHONE_STATE 	= "action_down_phone"
	MQTT_USER_ACTION_OPEN_DOOR_STATE 	= "action_open_door"
)
