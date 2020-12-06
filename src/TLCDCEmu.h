/*
	Tuner List CD Changer Emulator
	based on code from https://github.com/Tomasz-Mankowski/MeganeBT
*/

#ifndef _TLCDCEMU_H
#define _TLCDCEMU_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_avrc_api.h"
#include "esp_types.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "soc/timer_group_struct.h"

#include "esp32-hal-log.h"
#include "esp32-hal-uart.h"

#include "BluetoothA2DPSink.h"

#define LOG_TAG "TLCDCEmu"
#define BUFF_SIZE 256

//timer defs
#define TIMER_DIVIDER         16
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define TIMER_INTERVAL0_SEC   (1)
#define TIMER_INTERVAL1_SEC   (0.02)
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD      1

typedef enum{
	WAITING,
	CONFIRMED,
	TIMEOUT
}CDC_Wait_E;

typedef enum{
	WAIT_BOOT,
	BOOT_SEQUENCE,
	WAIT_HU_VERSION,
	CONFIRM_HU_VERSION,

	RECEIVED_PLAY,
	RECEIVED_PAUSE,
	RECEIVED_STANDBY,
	RECEIVED_CD_CHANGE,
	RECEIVED_NEXT,
	RECEIVED_PREV,

	OPERATE_STANDBY,
	OPERATE_PAUSED,
	OPERATE_PREPARE_PLAY,
	OPERATE_PLAYING
}CDC_State;

typedef struct {
	int type;
	int timer_group;
	int timer_idx;
	uint64_t timer_counter_value;
} timer_event_t;

class TLCDCEmu
{
	public:
		TLCDCEmu();
		~TLCDCEmu();
		void init();
		void talk();

	private:
		uint8_t CDC_SendPacket(uint8_t *data, uint8_t length, uint8_t retries);
		static uint8_t CDC_checksum(const uint8_t *data, uint8_t length);
		void CDC_ConfirmSongChange();
		static void readHU(const uint8_t *data, uint16_t length);
		void fakePlay();
		static void uart_event_task(void *pvParameters);
		uint8_t con1, con2, con3, con4;
		
		uart_config_t uart_config;
		xTaskHandle uartHandle;

		//timer
		static void tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec);
		static void timer_evt_task(void *arg);
};

#ifdef __cplusplus
}
#endif

#endif

