/*@file   ESPNOW_tank.c
  @brief  simple implementation of ESPNOW protocol (tank end esp32 device)
  @author bheesma-10
*/

#include <stdio.h>
//#include <stdbool.h>
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
uint8_t sender_MAC[]   = {0x3c,0x61,0x05,0x30,0x60,0x61};

static uint8_t received_successfully;
static uint8_t timeout=false;

/*led pin definition and pin mask*/
#define GPIO_OUTPUT_LED       GPIO_NUM_2
#define GPIO_OUTPUT_PIN_SEL   (1ULL<<GPIO_OUTPUT_LED)

/*input float sensor pin*/
#define GPIO_INPUT_FLOAT       GPIO_NUM_4
#define GPIO_INPUT_PIN_SEL    (1ULL<<GPIO_INPUT_FLOAT)

const uint8_t GPIO_PIN_HIGH = 1;
const uint8_t GPIO_PIN_LOW = 0;

const char* TAG = "ESP32-ESP_NOW";

/*Queue handle*/
xQueueHandle espnow_queue;
xQueueHandle regular_queue;
xQueueHandle isr_queue;

/*Timer Handle*/
TimerHandle_t espnow_timer;
TimerHandle_t regular_timer;

/**
  * @brief  initialize gpio pin
  * @param  None
  * @retval None
  */
void gpio_init(void){
    gpio_config_t pin_conf;
    gpio_config_t float_pin_conf;

    /*configure output led pin*/
    pin_conf.pin_bit_mask=GPIO_OUTPUT_PIN_SEL;
    pin_conf.mode=GPIO_MODE_OUTPUT;
    pin_conf.pull_up_en=false;
    pin_conf.pull_down_en=false;
    pin_conf.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&pin_conf);

    /*configure input float pin*/
    float_pin_conf.pin_bit_mask=GPIO_INPUT_PIN_SEL;
    float_pin_conf.mode=GPIO_MODE_INPUT;
    float_pin_conf.pull_up_en=false;
    float_pin_conf.pull_down_en=false;
    float_pin_conf.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&float_pin_conf);



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
    for(int i=0;i<60000;i++){}             //some delay
    led_off();

    evt.id = ESPNOW_LED_TASK;

    xQueueReset(espnow_queue);
    if(xQueueSend(espnow_queue,&evt,200)!=pdTRUE){
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
    espnow_event_t evt;
    espnow_event_recv_cb_t recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    
    memcpy(recv_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb.data = data;
    recv_cb.data_len = len;

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
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}
/**
  * @brief  Timer Callback
  * @param  timer handle
  * @retval None
  * @note   espnow_timer - activate timeout after 50s if nothing received and go back to sleep else stop timer
  *         regular_timer - checks for float sensors feedback every 10s and sends status over isr_queue to regular task
  */
void vTimerCallback(TimerHandle_t xTimer){
if(xTimer==espnow_timer){
  static uint8_t count;
  
  if((received_successfully==0) && (count==5)){
       timeout=true;
   }
  else if(received_successfully==1){
       received_successfully=0;
       count=0;
       timeout=false;
       /*stop timer*/
       if(xTimerStop(espnow_timer,pdMS_TO_TICKS(200))!=pdPASS){
             ESP_LOGI(TAG,"error stopping timer");
       }
   } 
  else{
       count++;
  }
}   
else{
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   uint8_t status=0;
   uint8_t float_sensor = gpio_get_level(GPIO_INPUT_FLOAT);
   if(float_sensor==GPIO_PIN_HIGH){
      status = SEND_STOP;
   }
   else{
      status = RUNNING;
   }

   xQueueSendFromISR( isr_queue, &status, &xHigherPriorityTaskWoken );
}
}

/**
  * @brief  regular main task
  * @param  task parameters
  * @retval None
  * @note   handles the process at tank end. begins with being in receiving mode until timeout occurs or {"motor":"start"}
  * is received from sender end esp32. {"status":"ok"} is sent back as acknowledgement.
  * once float sensor indicates that tank is filled, {"motor":"stopped"} is sent and ack is received for the same.
*/
void regular_task(void* pvParameters){

	/*timer to check for incoming data*/
	if(xTimerStart(espnow_timer,0)!=pdPASS){
		ESP_LOGI(TAG,"error starting timer");
	}

    const TickType_t xtask_delay = pdMS_TO_TICKS(30000);
    TickType_t xtaskWakeTime;
	char received_data[20] = {0};
	static uint8_t status;
	const cJSON *motor_start = NULL;
	const cJSON *ack = NULL;
        espnow_send_param_t send_param ;
       
	

while(1){

       if(timeout){
           ESP_LOGI(TAG,"timeout.............");
           espnow_deinit(&send_param);
	       esp_wifi_stop();
           esp_deep_sleep_start();
	}

    /*received data from espnow task - checks the data received and act accordingly*/
	while(xQueueReceive(regular_queue,received_data,2000) == pdTRUE){
        for(int datalen=0;datalen<sizeof(received_data);datalen++){
       	 ESP_LOGI(TAG,"%c",received_data[datalen]);
        }
		cJSON *esp_json = cJSON_ParseWithLength(received_data,sizeof(received_data));
		if(esp_json==NULL){
		 ESP_LOGE(TAG,"error");
		}
		
		motor_start = cJSON_GetObjectItemCaseSensitive(esp_json,"motor");
		if(cJSON_IsString(motor_start) && motor_start->valuestring!=NULL){
		 ESP_LOGI(TAG,"motor command:%s\n",motor_start->valuestring);
                 if(strstr(motor_start->valuestring,"start") != NULL){
                   ESP_LOGI(TAG,"motor started");
                   status = RUNNING;
                  /*start timer*/
                   if(xTimerStart(regular_timer,0)!=pdPASS){
		        ESP_LOGI(TAG,"error starting timer");
	           }

                   char* ack= "{\"status\":\"ok\"}";
                   memcpy(send_param.dest_mac,sender_MAC,sizeof(sender_MAC));
                   send_param.buffer = (uint8_t*)ack;
                   send_param.len = strlen(ack);
                                     
                   if(esp_now_send(send_param.dest_mac,send_param.buffer,send_param.len)!=ESP_OK){
                        ESP_LOGE(TAG, "Send error");
                   }
                 }
		}
		
	       	ack = cJSON_GetObjectItemCaseSensitive(esp_json,"status");
		if(cJSON_IsString(ack) && ack->valuestring!=NULL){
		 ESP_LOGI(TAG,"ack received:%s\n",ack->valuestring);
                 if(strstr(ack->valuestring,"ok") != NULL){
                   ESP_LOGI(TAG,"motor stopped");
                   status = STOPPED;
                  /*stop timer*/
                   if(xTimerStop(regular_timer,pdMS_TO_TICKS(200))!=pdPASS){
		        ESP_LOGI(TAG,"error stopping timer");
	           }
                 }
		}
	}

   /*different stages of the flow*/
   switch(status){

         case RUNNING:{
                      BaseType_t xTaskWokenByReceive = pdFALSE;
                      xQueueReceiveFromISR( isr_queue,( void * ) &status,&xTaskWokenByReceive);
                      }
                      break;
         case SEND_STOP:{
                      char* sending_payload= "{\"motor\":\"stop\"}";
                      memcpy(send_param.dest_mac,sender_MAC,sizeof(sender_MAC));
                      send_param.buffer = (uint8_t*)sending_payload;
                      send_param.len = strlen(sending_payload);
                                     
                      if(esp_now_send(send_param.dest_mac,send_param.buffer,send_param.len)!=ESP_OK){
                              ESP_LOGE(TAG, "Send error");
                      }
                      }
                      break;
         case STOPPED:{
                      espnow_deinit(&send_param);
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

                                         ESP_LOGI(TAG,"send task complete");
                                         ESP_LOGI(TAG,"data sent to device:");
                                         for(int mac_size=0;mac_size<ESP_NOW_ETH_ALEN;mac_size++){
                                         	ESP_LOGI(TAG,"%02X",evt.info.send_cb.mac_addr[mac_size]);
                                         }
                                         
                                   }
			                       break;
			case ESPNOW_RECV_CB:    {
                                                       
							            received_successfully = 1;
				                        ESP_LOGI(TAG,"data received");
				                        ESP_LOGI(TAG,"received data length:%d",evt.info.recv_cb.data_len);
				                        for(int datalen=0;datalen<evt.info.recv_cb.data_len;datalen++){
				                       	 ESP_LOGI(TAG,"%c",evt.info.recv_cb.data[datalen]);
                                                         
				                        }
				                        ESP_LOGI(TAG,"\r\n");

							/*send received data to regular task*/
							if(xQueueSend(regular_queue,(char*)evt.info.recv_cb.data,2000) != pdTRUE){
							 ESP_LOGW(TAG,"send queue fail");
							}
							
						
                                        /****************************************************/
                                        xTaskCreate(led_task,"LED_TASK",2500,(void*)500,5,NULL);
                                        /****************************************************/
                                        

                                    }
                            


                                   break;
                                
            default:
                                   break;
		
	    }
	  }
	      vTaskDelay(pdMS_TO_TICKS(100));	
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
    
    /*create queue for application*/
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "operation failed");
    }

    regular_queue = xQueueCreate(1,20*sizeof(char));
    if(regular_queue == NULL){
	ESP_LOGE(TAG,"operation failed");
    }

    isr_queue = xQueueCreate(1,sizeof(uint8_t));
    if(isr_queue == NULL){
	ESP_LOGE(TAG,"operation failed");
    }

    /*create timer*/
    espnow_timer = xTimerCreate("ESPNOW Timer",pdMS_TO_TICKS(10000),pdTRUE,(void*)0,vTimerCallback);
	
    regular_timer = xTimerCreate("Regular Timer",pdMS_TO_TICKS(10000),pdTRUE,(void*)0,vTimerCallback);


    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /*set primary key*/
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"PMK1233443433245"));

    /*add sender device address to peer list*/
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer,0,sizeof(esp_now_peer_info_t));
    peer->channel = 1;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr,sender_MAC,sizeof(sender_MAC));
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);


    /*get number of peers and peer data from stored list*/
    ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
    ESP_LOGI(TAG,"no of peers in peers list:%d",num_peers.total_num);
    peer =(esp_now_peer_info_t*) malloc(sizeof(esp_now_peer_info_t));
    for(int num_peer=0;num_peer<num_peers.total_num;num_peer++){
        esp_now_get_peer(sender_MAC,peer);
        
        ESP_LOGI(TAG,"channel:%d",peer->channel);
        ESP_LOGI(TAG,"peer address");
        for(int address_size=0;address_size<ESP_NOW_ETH_ALEN;address_size++){
            ESP_LOGI(TAG,"%02X\r",peer->peer_addr[address_size]);
        }
    } 
    free(peer);

    /*create task*/
    xTaskCreate(esp_now_task,"ESP now Task",5000,NULL,2,NULL);
    xTaskCreate(regular_task,"Regular Task",5000,NULL,3,NULL);
}
/**
  * @brief  checks for deep sleep wakeup reason
  * @param  None
  * @retval None
  */
void wakeup_reason(void){
	 switch (esp_sleep_get_wakeup_cause()) {
		case ESP_SLEEP_WAKEUP_TIMER: {
		    printf("Wake up from timer. \n");
		    wifi_init();
		    espnow_init();
		    break;
		}
		case ESP_SLEEP_WAKEUP_UNDEFINED:
		default:{
                    printf("Not a deep sleep reset\n");
		    esp_deep_sleep_start();
		}
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
    const int wakeup_time_sec = 60;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    /*check wakeup reason*/
    wakeup_reason();

}


























 
