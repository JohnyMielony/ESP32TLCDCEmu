/*
	Tuner List CD Changer Emulator
	based on code from https://github.com/Tomasz-Mankowski/MeganeBT
*/

#include "TLCDCEmu.h"

const uint8_t ACK[1] = {0xC5};

const uint8_t CDC_Payload_WaitBoot[3] = {0x11, 0x61, 0x06};
const uint8_t CDC_Payload_BootSequence_1[3] = {0x15, 0x00, 0x25};
const uint8_t CDC_Payload_BootSequence_2[6] = {0x20, 0x01, 0x03, 0x09, 0x15, 0x01};
const uint8_t CDC_Payload_BootSequence_3[2] = {0x25, 0x03};
const uint8_t CDC_Payload_BootSequence_4[5] = {0x26, 0x15, 0x00, 0x80, 0x80};
const uint8_t CDC_Payload_ConfirmHuVersion[2] = {0x62, 0x01};
const uint8_t CDC_Payload_ConfirmPlay[2] = {0x21, 0x05};
const uint8_t CDC_Payload_ConfirmPause[2] = {0x21, 0x03};
const uint8_t CDC_Payload_ConfirmStandby[2] = {0x21, 0x01};
const uint8_t CDC_Payload_ConfirmSongChange_1[4] = {0x27, 0x80, 0x01, 0x22};
const uint8_t CDC_Payload_ConfirmSongChange_2[2] = {0x21, 0x0A};
const uint8_t CDC_Payload_ConfirmSongChange_3[2] = {0x21, 0x05};
const uint8_t CDC_Payload_ConfirmSongChange_4[4] = {0x27, 0x15, 0x00, 0x22};

const uint8_t CDC_Payload_OperateStandby[6] = {0x20, 0x01, 0x03, 0x09, 0x15, 0x01};
const uint8_t CDC_Payload_OperatePreperePlay[6] = {0x20, 0x05, 0x03, 0x09, 0x15, 0x01};
const uint8_t CDC_Payload_OperatePaused[6] = {0x20, 0x03, 0x03, 0x09, 0x15, 0x01};
uint8_t CDC_Payload_OperatePlaying[11] = {0x47, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

static volatile uint8_t CDC_TX_buffer[32];
static volatile uint8_t CDC_RX_buffer[32];
static volatile uint8_t CDC_RX_Ptr;

static uint8_t RX_BUFF[64];

static volatile CDC_Wait_E CDC_Wait;
static volatile CDC_State CDC_CurrentState;
static volatile uint8_t CDC_SendSequence;
static volatile uint8_t CDC_PlaySequence;

static QueueHandle_t uart_queue;
static xQueueHandle timer_queue;

TLCDCEmu* pTLCDCEmu;

BluetoothA2DPSink btSink;

TLCDCEmu::TLCDCEmu(){
	pTLCDCEmu = this;
	uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_EVEN,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
}

TLCDCEmu::~TLCDCEmu(){
	uart_driver_delete(UART_NUM_2);
}

void TLCDCEmu::init(){
	ESP_LOGI(LOG_TAG,"INIT");
	CDC_RX_Ptr = 0;
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2,15,16,13,14)); //TX RX
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2,BUFF_SIZE,BUFF_SIZE,10,&uart_queue,0));
	xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

	SPDIFOut *out;
	out = new SPDIFOut();
	btSink.start((char*)"LAGUNA",out);

	//Timer
	timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
	tg0_timer_init(TIMER_1, TEST_WITH_RELOAD, TIMER_INTERVAL1_SEC);
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
}

void TLCDCEmu::talk(){
	switch(CDC_CurrentState){
		case WAIT_BOOT:
			ESP_LOGD(LOG_TAG,"WAIT_BOOT");
			if(CDC_SendPacket((uint8_t*)CDC_Payload_WaitBoot, 3, 1)){
				CDC_CurrentState = BOOT_SEQUENCE;
			}
			break;

		case BOOT_SEQUENCE:
			if (!con1){
				ESP_LOGD(LOG_TAG,"BOOT 1");
				con1 = CDC_SendPacket((uint8_t*)CDC_Payload_BootSequence_1, 3, 1);
				break;
			}
			if (!con2){
				ESP_LOGD(LOG_TAG,"BOOT 2");
				con2 = CDC_SendPacket((uint8_t*)CDC_Payload_BootSequence_2, 6, 1);
				break;
			}
			if (!con3){
				ESP_LOGD(LOG_TAG,"BOOT 3");
				con3 = CDC_SendPacket((uint8_t*)CDC_Payload_BootSequence_3, 2, 1);
				break;
			}
			if (!con4){
				ESP_LOGD(LOG_TAG,"BOOT 4");
				con4 = CDC_SendPacket((uint8_t*)CDC_Payload_BootSequence_4, 5, 1);
				break;
			}
			
			if(con1 && con2 && con3 && con4){
				ESP_LOGD(LOG_TAG,"WAIT_HU_VERSION");
				CDC_CurrentState = WAIT_HU_VERSION;
			}
			else{
				CDC_SendSequence = 0;
				CDC_CurrentState = WAIT_BOOT;
			}
			break;

		case WAIT_HU_VERSION:
			break;

		case CONFIRM_HU_VERSION:
			ESP_LOGD(LOG_TAG,"CONFIRM_HU_VERSION");
			CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmHuVersion, 2, 1);
			CDC_CurrentState = OPERATE_STANDBY;
			break;

		case RECEIVED_PLAY:
			ESP_LOGD(LOG_TAG,"RECEIVED_PLAY");
			timer_pause(TIMER_GROUP_0, (timer_idx_t)0);
			btSink.sendCommand(PLAY);
			CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmPlay, 2, 1);
			CDC_CurrentState = OPERATE_PREPARE_PLAY;
			break;

		case RECEIVED_PAUSE:
			ESP_LOGD(LOG_TAG,"RECEIVED_PAUSE");
			timer_pause(TIMER_GROUP_0, (timer_idx_t)0);
			btSink.sendCommand(PAUSE);
			CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmPause, 2, 1);
			CDC_CurrentState = OPERATE_PAUSED;
			break;

		case RECEIVED_CD_CHANGE:
			ESP_LOGD(LOG_TAG,"RECEIVED_CD_CHANGE");
			timer_pause(TIMER_GROUP_0, (timer_idx_t)0);
			CDC_CurrentState = OPERATE_PREPARE_PLAY;
			CDC_ConfirmSongChange();
			break;

		case RECEIVED_NEXT:
			ESP_LOGD(LOG_TAG,"RECEIVED_NEXT");
			timer_pause(TIMER_GROUP_0, (timer_idx_t)0);
			btSink.sendCommand(NEXT);
			CDC_CurrentState = OPERATE_PREPARE_PLAY;
			CDC_ConfirmSongChange();
			break;

		case RECEIVED_PREV:
			ESP_LOGD(LOG_TAG,"RECEIVED_PREV");
			timer_pause(TIMER_GROUP_0, (timer_idx_t)0);
			btSink.sendCommand(PREV);
			CDC_CurrentState = OPERATE_PREPARE_PLAY;
			CDC_ConfirmSongChange();
			break;

		case RECEIVED_STANDBY:
			ESP_LOGD(LOG_TAG,"RECEIVED_STANDBY");
			timer_pause(TIMER_GROUP_0, (timer_idx_t)0);
			btSink.sendCommand(PAUSE);
			CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmStandby, 2, 1);
			CDC_CurrentState = OPERATE_STANDBY;
			break;

		case OPERATE_PREPARE_PLAY:
			ESP_LOGD(LOG_TAG,"OPERATE_PREPARE_PLAY");
			timer_start(TIMER_GROUP_0, (timer_idx_t)0);
			CDC_CurrentState = OPERATE_PLAYING;
			break;

		case OPERATE_PAUSED:
		case OPERATE_STANDBY:
		case OPERATE_PLAYING:
			timer_start(TIMER_GROUP_0, (timer_idx_t)0);
			break;
	}
	
}

uint8_t TLCDCEmu::CDC_SendPacket(uint8_t *data, uint8_t length, uint8_t retries){
	if(length <= 28){
		for(int ret=0; ret<retries;ret++){
			CDC_TX_buffer[0] = 0x3D;
			CDC_TX_buffer[1] = CDC_SendSequence;
			CDC_TX_buffer[2] = length;
			for(uint8_t i=0; i<length; i++){
				CDC_TX_buffer[3+i] = data[i];
			}
			CDC_TX_buffer[3+length] = this->CDC_checksum((const uint8_t*)CDC_TX_buffer, length+3);
			//for (int ii=0;ii<length+4;ii++){
			//	ESP_LOGI(LOG_TAG,"-->%X",CDC_TX_buffer[ii]);
			//}
			uart_write_bytes(UART_NUM_2, (const char*)CDC_TX_buffer, length+4);
			uint8_t log_buff[length+4] = {};

			CDC_Wait = WAITING;
			uart_wait_tx_done(UART_NUM_2, 100);

			while(CDC_Wait == WAITING);

			if(CDC_Wait == CONFIRMED){
				CDC_SendSequence++;
				return 1;
			}
		}
	}

	return 0;
}

void TLCDCEmu::uart_event_task(void *pvParameters){
	uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUFF_SIZE);
    for(;;) {
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, BUFF_SIZE);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGD(LOG_TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
					readHU((const uint8_t*)dtmp,event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGE(LOG_TAG, "hw fifo overflow");
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGE(LOG_TAG, "ring buffer full");
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGE(LOG_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGE(LOG_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGE(LOG_TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(LOG_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void TLCDCEmu::readHU(const uint8_t *data, uint16_t length){
	//ESP_LOGI(LOG_TAG,"READ HU LENGTH: %d",length);
	
	for(int i=0;i<length;i++){
		ESP_LOGD(LOG_TAG,"<--%X",data[i]);
		CDC_RX_buffer[CDC_RX_Ptr] = data[i];
		CDC_RX_Ptr++;

		if(CDC_RX_buffer[0] == 0xC5){ // confirmation
			CDC_RX_Ptr = 0; //start transimison over
			CDC_Wait = CONFIRMED; // release waiting for confirmation
		}
		else if(CDC_RX_buffer[0] != 0x3D){ //if not a new incoming packet header drop transmission
			CDC_RX_Ptr = 0;
		}

		if(CDC_RX_Ptr >= 4){ //handle incoming transmission
			if(CDC_RX_Ptr == CDC_RX_buffer[2] + 4){
				if(CDC_RX_buffer[CDC_RX_Ptr-1] == CDC_checksum((const uint8_t*)CDC_RX_buffer, CDC_RX_buffer[2]+3)){
					//ESP_LOG_BUFFER_HEX_LEVEL(LOG_TAG, (const uint8_t*)CDC_RX_buffer, CDC_RX_buffer[2] + 4, ESP_LOG_INFO);
					uart_write_bytes(UART_NUM_2, (const char*)ACK, 1);
					uart_wait_tx_done(UART_NUM_2, 100);
					ESP_LOGD(LOG_TAG,"-->C5");
					if(CDC_RX_buffer[2] == 7 && CDC_RX_buffer[3] == 0x31){
						CDC_CurrentState = CONFIRM_HU_VERSION;
						ESP_LOGD(LOG_TAG,"RECEIVED CONFIRM_HU_VERSION");
					}

					if(CDC_RX_buffer[2] == 1){
						if(CDC_RX_buffer[3] == 0x13){
							CDC_CurrentState = RECEIVED_PLAY;
						}

						if(CDC_RX_buffer[3] == 0x1C){
							CDC_CurrentState = RECEIVED_PAUSE;
						}

						if(CDC_RX_buffer[3] == 0x19){
							CDC_CurrentState = RECEIVED_STANDBY;
						}

						if(CDC_RX_buffer[3] == 0x24){
							CDC_CurrentState = RECEIVED_CD_CHANGE;
						}
					}

					if(CDC_RX_buffer[2] == 2){
						if(CDC_RX_buffer[3] == 0x17 && CDC_RX_buffer[4] == 0x01){
							CDC_CurrentState = RECEIVED_NEXT;
						}

						if(CDC_RX_buffer[3] == 0x2C && CDC_RX_buffer[4] == 0xFF){
							//ENTERING CDC MODE
							ESP_LOGI(LOG_TAG,"ENTERING CDC MODE");
						}

						if(CDC_RX_buffer[3] == 0x2C && CDC_RX_buffer[4] == 0x00){
							//EXITING CDC MODE
							ESP_LOGI(LOG_TAG,"EXITING CDC MODE");
						}
					}

					if(CDC_RX_buffer[2] == 3){
						if(CDC_RX_buffer[3] == 0x22 && CDC_RX_buffer[4] == 0x01 && CDC_RX_buffer[5] == 0x02){
							CDC_CurrentState = RECEIVED_PREV;
						}
					}
				}
				CDC_RX_Ptr = 0;
			}
			else if(CDC_RX_Ptr > CDC_RX_buffer[2] + 4){ //if received data to long, drop transmission
				CDC_RX_Ptr = 0;
			}
		}

		if(CDC_RX_Ptr == 32){
			CDC_RX_Ptr = 0;
		}
	}
}

void TLCDCEmu::fakePlay(){
	switch(CDC_CurrentState)
	{
		case OPERATE_STANDBY:
			ESP_LOGD(LOG_TAG,"fakePlay OPERATE_STANDBY");
			CDC_SendPacket((uint8_t*)CDC_Payload_OperateStandby, 6, 1);
			break;

		case OPERATE_PREPARE_PLAY:
			ESP_LOGD(LOG_TAG,"fakePlay OPERATE_PREPARE_PLAY");
			CDC_SendPacket((uint8_t*)CDC_Payload_OperatePreperePlay, 6, 1);
			break;

		case OPERATE_PLAYING:
			ESP_LOGD(LOG_TAG,"fakePlay OPERATE_PLAYING");
			CDC_Payload_OperatePlaying[9] = CDC_PlaySequence;
			CDC_SendPacket((uint8_t*)CDC_Payload_OperatePlaying, 11, 1);
			CDC_PlaySequence++;
			break;

		case OPERATE_PAUSED:
			ESP_LOGD(LOG_TAG,"fakePlay OPERATE_PAUSED");
			CDC_SendPacket((uint8_t*)CDC_Payload_OperatePaused, 6, 1);
			break;

		default:
			ESP_LOGD(LOG_TAG,"fakePlay default");
			break;
	}
}

uint8_t TLCDCEmu::CDC_checksum(const uint8_t *data, uint8_t length){
	uint8_t res = 0;
	for (uint8_t i=0;i<length;i++){
		res = res ^ data[i];
	}
	return res;
}

void TLCDCEmu::CDC_ConfirmSongChange(){
	ESP_LOGD(LOG_TAG,"CDC_ConfirmSongChange");
	this->CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmSongChange_1, 4, 1);
	this->CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmSongChange_2, 2, 1);
	this->CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmSongChange_3, 2, 1);
	this->CDC_SendPacket((uint8_t*)CDC_Payload_ConfirmSongChange_4, 4, 1);
	CDC_PlaySequence = 0;
}

//TIMER
static void IRAM_ATTR timer_group0_isr(void *para){
	int timer_idx = (int) para;

    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;

    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
		timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    }
	else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1){
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
	}
	else{
		evt.type = -1;
		ESP_LOGD(LOG_TAG,"Not supported event type");
    }

    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

void TLCDCEmu::tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec){
	timer_config_t timer_config;
	timer_config.divider = TIMER_DIVIDER;
	timer_config.counter_dir = TIMER_COUNT_UP;
	timer_config.counter_en = TIMER_PAUSE;
	timer_config.alarm_en = TIMER_ALARM_EN;
	timer_config.intr_type = TIMER_INTR_LEVEL;
	timer_config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, (timer_idx_t)timer_idx, &timer_config);

    timer_set_counter_value(TIMER_GROUP_0, (timer_idx_t)timer_idx, 0x00000000ULL);

    timer_set_alarm_value(TIMER_GROUP_0, (timer_idx_t)timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, (timer_idx_t)timer_idx);
    timer_isr_register(TIMER_GROUP_0, (timer_idx_t)timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

void TLCDCEmu::timer_evt_task(void *arg){
	while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        uint64_t task_counter_value;
        timer_get_counter_value((timer_group_t)evt.timer_group, (timer_idx_t)evt.timer_idx, &task_counter_value);
		ESP_LOGD(LOG_TAG,"TIMER %d PING %.8f s",evt.timer_idx, (double)task_counter_value/TIMER_SCALE);
		switch(evt.timer_idx){
			case 0:
				pTLCDCEmu->fakePlay();
				break;
			
			case 1:
				timer_pause(TIMER_GROUP_0, (timer_idx_t)1);
				CDC_Wait = TIMEOUT;
				ESP_LOGI(LOG_TAG,"TIMEOUT");
				break;
			
			default:
				break;
		}
    }
}
