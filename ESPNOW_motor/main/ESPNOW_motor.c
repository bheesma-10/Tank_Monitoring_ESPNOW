/*@file   ESPNOW_motor.c
  @brief  implementation of ESPNOW protocol (motor end esp32 device)
  @author bheesma-10
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/gpio.h"
#include "ESPNOW.h"
#include "cJSON.h"

/*receiver and sender ESP32 mac addresses(change these as per your device)*/
static const uint8_t receiver_MAC[] = {0x30,0xc6,0xf7,0x29,0x24,0x01};

uint8_t sent_successfully=0;     //flag to confirm that data is sent successfully
static uint8_t start_sending=1;  //initially transmit data

/*data to be sent and acknowledgement to be received*/
const char sending_payload[] = "{\"motor\":\"start\"}";
const char* ack = "{\"status\":\"ok\"}";

/*led pin definition and pin mask*/
#define GPIO_OUTPUT_LED              GPIO_NUM_2
#define GPIO_OUTPUT_PIN_SEL          (1ULL<<GPIO_OUTPUT_LED)

/*Input button to initiate tank filling procedure*/
#define GPIO_INPUT_BUTTON            GPIO_NUM_33
#define GPIO_INPUT_PIN_SEL           (1ULL<<GPIO_INPUT_BUTTON)

/*Output pin for relay control*/
#define GPIO_OUTPUT_RELAY            GPIO_NUM_4
#define GPIO_OUTPUT_RELAY_PIN_SEL    (1ULL<<GPIO_OUTPUT_RELAY)

const uint8_t GPIO_PIN_HIGH = 1;
const uint8_t GPIO_PIN_LOW = 0;
const char* TAG = "ESP32-ESP_NOW";

volatile uint8_t received_mac;

/*Queue handle*/
xQueueHandle espnow_queue;
xQueueHandle regular_queue;

/*Timer handle*/
TimerHandle_t espnow_timer;

/**
  * @brief  initialize gpio pin
  * @param  None
  * @retval None
  */
void gpio_init(void){
    gpio_config_t pin_conf;
    gpio_config_t relay_pin_conf;

    /*configure output led pin*/
    pin_conf.pin_bit_mask=GPIO_OUTPUT_PIN_SEL;
    pin_conf.mode=GPIO_MODE_OUTPUT;
    pin_conf.pull_up_en=false;
    pin_conf.pull_down_en=false;
    pin_conf.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&pin_conf);

    /*configure output relay pin*/
    relay_pin_conf.pin_bit_mask=GPIO_OUTPUT_RELAY_PIN_SEL;
    relay_pin_conf.mode=GPIO_MODE_OUTPUT;
    relay_pin_conf.pull_up_en=false;
    relay_pin_conf.pull_down_en=false;
    relay_pin_conf.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&relay_pin_conf);
}

/**
  * @brief  function to turn led off
  * @param  None
  * @retval None
  */
static void led_off(void){
    ESP_ERROR_CHECK(gpio_set_level(GPIO_OUTPUT_LED, 0));
}


/**
  * @brief  function to turn led on
  * @param  None
  * @retval None
  */
static void led_on(void){
    ESP_ERROR_CHECK(gpio_set_level(GPIO_OUTPUT_LED, 1));
}

/**
  * @brief  task for led operation  after transmitting data
  * @param  task parameters 
  * @retval None
  */
void led_task(void* pvParameters){
    espnow_event_t evt;

    uint32_t delay = (uint32_t)pvParameters;
    
    gpio_init();
    led_on();
    for(int i=0;i<60000;i++){}          //some delay
    led_off();

    evt.id = ESPNOW_LED_TASK; 
    
    xQueueReset(espnow_queue);
    if(xQueueSend(espnow_queue,&evt,200)!=pdPASS){
        ESP_LOGW(TAG,"send queue fail");
    }

    vTaskDelay(delay/portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}


/**
  * @brief  sending callback of ESPNOW 
  * @param  mac address of sending device, status of the transmission 
  * @retval None
  */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    

    espnow_event_t evt;
    espnow_event_send_cb_t send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    
    memcpy(send_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb.status = status;

    evt.id = ESPNOW_SEND_CB;
    evt.info.send_cb = send_cb;

    if (xQueueSend(espnow_queue, &evt, 200) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

/**
  * @brief  receiving callback of ESPNOW 
  * @param  mac address of received device, data received and length of received data
  * @retval None
  */
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    sent_successfully=1;            //until you receive an ack, transmission is not considered successful

    espnow_event_t evt;
    espnow_event_recv_cb_t recv_cb;

    memcpy(recv_cb.mac_addr,mac_addr,ESP_NOW_ETH_ALEN);
    recv_cb.data_len = len;
    recv_cb.data = data;

    evt.id = ESPNOW_RECV_CB;
    evt.info.recv_cb = recv_cb;
    
    if (xQueueSend(espnow_queue, &evt, 200) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        
    }
}

/**
  * @brief  receiving callback of ESPNOW 
  * @param  mac address of received device, data received and length of received data
  * @retval None
  */
static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

/**
  * @brief  Timer Callback
  * @param  timer handle
  * @retval None
  * @note   keeps a track if data sent successfully or not at interval of 500ms
  */
void vTimerCallback(TimerHandle_t xTimer){

    if(!sent_successfully){
        start_sending=1;
    }
    else{
        start_sending=0;
    }
}

/**
  * @brief  regular main task
  * @param  task parameters
  * @retval None
  * @note   handles the process at motor end. begins with sending {"motor":"start"} command at interval of 30s until 
  * ack is received from receiver end esp32. once {"status":"ok"} is received, only then motor starts.
  * on receiving {"motor":"stopped"}, ack is sent back and motor is stopped by switching relay back to it's original 
  * position.
*/
void regular_task(void* pvParameters){

    /*some variables*/
    const TickType_t xtask_delay = pdMS_TO_TICKS(30000);
	char received_data[20] = {0};
	static uint8_t status = SEND_START;
	const cJSON *motor_stop = NULL;
	const cJSON *ack = NULL;
    TickType_t xtaskWakeTime;

   espnow_send_param_t *send_param  = (espnow_send_param_t* )pvParameters;

   ESP_LOGI(TAG,"mac address in task:%02X:%02X:%02X:%02X:%02X:%02X",send_param->dest_mac[0],send_param->dest_mac[1],send_param->dest_mac[2],send_param->dest_mac[3],send_param->dest_mac[4],send_param->dest_mac[5]);
   ESP_LOGI(TAG,"buffer length:%d",send_param->len);
   for(int size=0;size<send_param->len;size++){
      ESP_LOGI(TAG,"%c",send_param->buffer[size]);
   }
   /*start timer*/
   if(xTimerStart(espnow_timer,0)!=pdPASS){
 	  ESP_LOGI(TAG,"error in starting timer2");
   }
      

while(1){

    /*received data from espnow task - checks the data received and act accordingly*/
	while(xQueueReceive(regular_queue,received_data,200) == pdTRUE){
		cJSON *esp_json = cJSON_ParseWithLength(received_data,strlen(received_data));
		if(esp_json==NULL){
		 ESP_LOGE(TAG,"error");
		}
		

		motor_stop = cJSON_GetObjectItemCaseSensitive(esp_json,"motor");
		if(cJSON_IsString(motor_stop) && motor_stop->valuestring!=NULL){
		 ESP_LOGI(TAG,"motor command:%s\n",motor_stop->valuestring);
                 if(strstr(motor_stop->valuestring,"stop") != NULL){
                   ESP_LOGI(TAG,"motor stopping");
                   status = STOPPED;
                  
                   char* stop_ack= "{\"status\":\"ok\"}";
                   memcpy(send_param->dest_mac,receiver_MAC,sizeof(receiver_MAC));
                   send_param->buffer = (uint8_t*)stop_ack;
                   send_param->len = strlen(stop_ack);
                                     
                   if(esp_now_send(send_param->dest_mac,send_param->buffer,send_param->len)!=ESP_OK){
                        ESP_LOGE(TAG, "Send error");
                   }
                  
                   vTaskDelay(pdMS_TO_TICKS(100));
                 }
		}
		
       	ack = cJSON_GetObjectItemCaseSensitive(esp_json,"status");
		if(cJSON_IsString(ack) && ack->valuestring!=NULL){
		         ESP_LOGI(TAG,"ack received:%s\n",ack->valuestring);
                 if(strstr(ack->valuestring,"ok") != NULL){
                   ESP_LOGI(TAG,"motor is running");
                   status = RUNNING;

                    /****************************************************/
                    xTaskCreate(led_task,"LED_TASK",2500,(void*)500,5,NULL);
                    /****************************************************/
                 }
		}
	}


   /*different stages of the flow*/
   switch(status){

         case SEND_START:{
                        /*stop sending once received succesfully*/
                         if(start_sending){
                           start_sending=0;
                           if(esp_now_send(send_param->dest_mac,(uint8_t*)sending_payload,send_param->len)!=ESP_OK){
                             ESP_LOGE(TAG, "Send error");
                            }
                         }
            
                      }                  
                      break;
         case RUNNING:{
                          /*turn motor on*/
                          gpio_set_level(GPIO_OUTPUT_RELAY, GPIO_PIN_HIGH);
                      }
                      break;
         case STOPPED:{
                         /*turn motor off*/
                          gpio_set_level(GPIO_OUTPUT_RELAY, GPIO_PIN_LOW);
                      
                          ESP_LOGI(TAG,"going to sleep..............");
                          espnow_deinit(send_param);
	                      esp_wifi_stop();
                          esp_deep_sleep_start();
                      }
                      break;
         default:
                      break;
                      
    }

    xtaskWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xtaskWakeTime,xtask_delay);
}
vTaskDelete(NULL);
}

/**
  * @brief  ESPNOW main task
  * @param  task parameters
  * @retval None
  * @note   indicates if data sent or received. sends received data to regular task
  */
void esp_now_task(void* pvParameters){

       espnow_event_t evt;
       


 for(;;){
	
    while(xQueueReceive(espnow_queue,&evt,200)==pdTRUE){
		switch(evt.id){
			case ESPNOW_SEND_CB:   {
                                        ESP_LOGI(TAG,"send cb task complete");
                                        
                                   }
			                       break;
			case ESPNOW_RECV_CB:    {

				        ESP_LOGI(TAG,"receive cb task");
                                        ESP_LOGI(TAG,"%d",evt.info.recv_cb.data_len);   
                                            

                                        /*send received data to regular task*/
					if(xQueueSend(regular_queue,(char*)evt.info.recv_cb.data,200) != pdTRUE){
							 ESP_LOGW(TAG,"send queue fail");
					}  

                                    }

                                   break;
            default:
                                   break;
		
	    }
	  }
        vTaskDelay(pdMS_TO_TICKS(10));	  
    }
    
    vTaskDelete(NULL);

}


/**
  * @brief  Wifi mode init
  * @param  None
  * @retval None
  * @note   WiFi should start before using ESPNOW. also long range is enabled so TX power is high and bandwidth low
  */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}


/**
  * @brief  ESPNOW init
  * @param  None
  * @retval None
  * @note   initialize queue size, network params, peer list update and generating sending data
  */
void espnow_init(void){

    
    esp_now_peer_num_t num_peers;
    espnow_send_param_t *send_param;

    /*create queue for application*/
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
    }
    regular_queue = xQueueCreate(1,20*sizeof(char));
    if(regular_queue == NULL){
	ESP_LOGE(TAG,"operation failed");
    }
    /*create timer for application(recursive timer)*/
    espnow_timer = xTimerCreate("ESPNOW timer",pdMS_TO_TICKS(500),pdTRUE,(void*)0,vTimerCallback);

	
	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /*set primary key*/
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"PMK1233443433245"));

    /*add receiver address to peer list*/
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer,0,sizeof(esp_now_peer_info_t));
    peer->channel = 1;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr,receiver_MAC,sizeof(receiver_MAC));
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /*get number of peers and peer data from stored list*/
    ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
    ESP_LOGI(TAG,"no of peers in peers list:%d",num_peers.total_num);
    peer =(esp_now_peer_info_t*) malloc(sizeof(esp_now_peer_info_t));
    for(int num_peer=0;num_peer<num_peers.total_num;num_peer++){
        esp_now_get_peer(receiver_MAC,peer); 
        ESP_LOGI(TAG,"channel:%d",peer->channel);
        ESP_LOGI(TAG,"peer address");
        for(int address_size=0;address_size<ESP_NOW_ETH_ALEN;address_size++){
            ESP_LOGI(TAG,"%02X\r",peer->peer_addr[address_size]);
        }
    }
    free(peer);


    /* Initialize sending parameters. */ 
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
    }
    send_param->unicast = true;
    send_param->count = sizeof(sending_payload);//CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = 1000;
    send_param->len = sizeof(sending_payload);
    send_param->buffer = &sending_payload[0];
    ESP_LOGI(TAG,"before task buffer");
    for(int size=0;size<send_param->len;size++){
        ESP_LOGI(TAG,"%c",send_param->buffer[size]);
    }
    memcpy(send_param->dest_mac, receiver_MAC, ESP_NOW_ETH_ALEN);
    ESP_LOGI(TAG,"mac address:%02X:%02X:%02X:%02X:%02X:%02X",send_param->dest_mac[0],send_param->dest_mac[1],send_param->dest_mac[2],send_param->dest_mac[3],send_param->dest_mac[4],send_param->dest_mac[5]);



    /*create task*/
    xTaskCreate(esp_now_task,"ESP now Task",5000,NULL,2,NULL);
    xTaskCreate(regular_task,"Regular Task",5000,(void*)send_param,3,NULL);

}


/**
  * @brief  checks for deep sleep wakeup reason
  * @param  None
  * @retval None
  * @note   only wakes up by external interrupt from button
  */
void wakeup_reason(void){
	 switch (esp_sleep_get_wakeup_cause()) {
		case ESP_SLEEP_WAKEUP_EXT1: {
		    printf("Wake up from GPIO\n");
                    start_sending=1;
		    wifi_init();
		    espnow_init();
		    break;
		}
		case ESP_SLEEP_WAKEUP_UNDEFINED:
		default:
		    printf("still sleeping.......zzzzzzzzzzzzzzzzzzzzzzzzzzzzzz\n");
		    esp_deep_sleep_start();
	    }
}

/**
  * @brief  main application
  * @param  None
  * @retval None
  */
void app_main(void)
{
	// Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    /*set wakeup method*/
    esp_sleep_enable_ext1_wakeup(GPIO_INPUT_PIN_SEL,ESP_EXT1_WAKEUP_ANY_HIGH);

    /*check wakeup reason*/
    wakeup_reason();

}
