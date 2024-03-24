#include <stdio.h>
#include <stdarg.h>

#include "driver/i2s.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_avrc_api.h"
#include "esp_a2dp_api.h"

#include "nvs_flash.h"

#include "freertos/queue.h"
#include "freertos/task.h"


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


static const i2s_config_t I2S_Config = {
	.mode = I2S_MODE_MASTER | I2S_MODE_TX,
	.sample_rate = 44100,
	.bits_per_sample = 16,
	.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	.communication_format = I2S_COMM_FORMAT_STAND_MSB,
	.intr_alloc_flags = 0,
	.dma_buf_count = 8,
	.dma_buf_len = 64,
	.use_apll = false,
	.tx_desc_auto_clear = true
};

typedef void BluetoothCallbackFunction(u16 event, void *param);
typedef struct BluetoothMessage {
	u16 kind;
	u16 event;
	BluetoothCallbackFunction *callback;
	void *param;
} BluetoothMessage;


static esp_avrc_rn_evt_cap_mask_t CapabilityMask;

static esp_a2d_audio_state_t AudioState = ESP_A2D_AUDIO_STATE_STOPPED;
static u8 Volume;


static void die(char const *format, ...) {
	va_list args;
	va_start(args, format);
	printf("ERROR:");
	vprintf(format, args);
	printf("\nAborting\n");
	va_end(args);
	abort();
}

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
	switch (event) {
	case ESP_BT_GAP_AUTH_CMPL_EVT:
		if (param->auth_cmpl.stat != ESP_BT_STATUS_SUCCESS)
			die("Bluetooth authentication failed: %d", param->auth_cmpl.stat);
	break; // ESP_BT_GAP_AUTH_CMPL_EVT

	// SSP event
	case ESP_BT_GAP_CFM_REQ_EVT:
		printf("Pairing: please confirm the following code matches the one on your phone: %d", param->cfm_req.num_val);
		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
	break;

	default: break;
	}
}

#define GET_METADATA 2
#define TRACK_CHANGED 3
#define PLAYBACK_TOGGLED 4
#define PLAY_POSITION_CHANGED 5
static void start_new_track(void) {
	esp_avrc_ct_send_metadata_cmd(GET_METADATA, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE);

	if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &CapabilityMask, ESP_AVRC_RN_TRACK_CHANGE))
		esp_avrc_ct_send_register_notification_cmd(TRACK_CHANGED, ESP_AVRC_RN_TRACK_CHANGE, 0);
}

static void toggle_playback(void) {
	if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &CapabilityMask, ESP_AVRC_RN_PLAY_STATUS_CHANGE))
		esp_avrc_ct_send_register_notification_cmd(PLAYBACK_TOGGLED, ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
}

static void change_play_position(void) {
	if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &CapabilityMask, ESP_AVRC_RN_PLAY_POS_CHANGED))
		esp_avrc_ct_send_register_notification_cmd(PLAY_POSITION_CHANGED, ESP_AVRC_RN_PLAY_POS_CHANGED, 0);
}

#define GET_CAPABILITIES 0
static void avrc_controller_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
	switch (event) {
	case ESP_AVRC_CT_CONNECTION_STATE_EVT:
		if (param->conn_stat.connected)
			esp_avrc_ct_send_get_rn_capabilities_cmd(GET_CAPABILITIES);
		else
			CapabilityMask.bits = 0;
	break; // ESP_AVRC_CT_CONNECTION_STATE_EVT

	case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
		switch (param->change_ntf.event_id) {
		case ESP_AVRC_RN_TRACK_CHANGE:
			start_new_track();
		break; // ESP_AVRC_RN_TRACK_CHANGE

		case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
			toggle_playback();
		break; // ESP_AVRC_RN_PLAY_STATUS_CHANGE

		case ESP_AVRC_RN_PLAY_POS_CHANGED:
			change_play_position();
		break; // ESP_AVRC_RN_PLAY_POS_CHANGED
		}
	break; // ESP_AVRC_CT_CHANGE_NOTIFIY_EVT

	case ESP_AVRC_CT_METADATA_RSP_EVT:
		printf("Metadata: %.*s\n", param->meta_rsp.attr_length, param->meta_rsp.attr_text);
	break; // ESP_AVRC_CT_METADATA_RSP_EVT

	case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
		CapabilityMask.bits = param->get_rn_caps_rsp.evt_set.bits;

		start_new_track();
		toggle_playback();
		change_play_position();
	break; // ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT

	default: break;
	}
}

static void avrc_target_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
	switch (event) {
	case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
		Volume = param->set_abs_vol.volume;
	break; // ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT

	default: break;
	}
}

static void a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
	switch (event) {
	case ESP_A2D_CONNECTION_STATE_EVT:
		if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

		if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
			esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
	break; // ESP_A2D_CONNECTION_STATE_EVT

	case ESP_A2D_AUDIO_STATE_EVT:
		AudioState = param->audio_stat.state;
	break; // ESP_A2D_AUDIO_STATE_EVT

	case ESP_A2D_AUDIO_CFG_EVT:
		if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
			int sample_rate = 16000;
			char mask = param->audio_cfg.mcc.cie.sbc[0];
			if (mask & (0x01 << 6))
				sample_rate = 32000;
			else if (mask & (0x01 << 5))
				sample_rate = 44100;
			else if (mask & (0x01 << 4))
				sample_rate = 48000;

			i2s_set_clk(0, sample_rate, 16, 2);
		} else {
			die("Stream type not supported right now.");
		}
	break; // ESP_A2D_AUDIO_CFG_EVT

	default: break;
	}
}

static void sink_data_callback(u8 const *data, u32 length) {
	size_t bytes_written;
	printf("Received data: %d\n", length);
	i2s_write(0, data, length, &bytes_written, portMAX_DELAY);
	// NOTE: Try again?
}


void app_main(void) {
	esp_err_t error = nvs_flash_init();
	if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
		die("Not enough memory to initialize NVS.");

	i2s_driver_install(I2S_NUM_0, &I2S_Config, 0, NULL);
	i2s_pin_config_t pin_config = {
		.bck_io_num = 26,
		.ws_io_num = 22,
		.data_out_num = 25,
		.data_in_num = -1
	};
	i2s_set_pin(I2S_NUM_0, &pin_config);

	esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

	error = esp_bt_controller_init(&bt_config);
	if (error != ESP_OK) die("Bluetooth initialisazion failed: %s.", esp_err_to_name(error));
	error = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
	if (error != ESP_OK) die("Bluetooth activation failed: %s.", esp_err_to_name(error));
	error = esp_bluedroid_init();
	if (error != ESP_OK) die("Bluedroid initialization failed: %s.", esp_err_to_name(error));
	error = esp_bluedroid_enable();
	if (error != ESP_OK) die("Bluedroid activation failed: %s.", esp_err_to_name(error));

	esp_bt_dev_set_device_name("Pämbers");

	esp_bt_gap_register_callback(gap_callback);

	esp_avrc_ct_init();
	esp_avrc_ct_register_callback(avrc_controller_callback);
	if (esp_avrc_tg_init() != ESP_OK) die("Bluetooth AVRC initialization failed.");
	esp_avrc_tg_register_callback(avrc_target_callback);

	esp_avrc_rn_evt_cap_mask_t event_mask = {0};
	esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &event_mask, ESP_AVRC_RN_VOLUME_CHANGE);
	esp_avrc_tg_set_rn_evt_cap(&event_mask);

	esp_a2d_register_callback(a2d_callback);
	esp_a2d_sink_register_data_callback(sink_data_callback);
	esp_a2d_sink_init();

	esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(u8));
}
