#include "esp_event.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "veml7700.h"
#include "HYGRO.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

/* Set the SSID and Password via project configuration, or can set directly here */
#define DEFAULT_SSID "DIGIFIBRA-b7xc"
#define DEFAULT_PWD "3kb56hGE9Y"

#if CONFIG_EXAMPLE_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_EXAMPLE_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_EXAMPLE_SCAN_METHOD*/

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_EXAMPLE_SORT_METHOD*/

#if CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD
#define DEFAULT_RSSI CONFIG_EXAMPLE_FAST_SCAN_MINIMUM_SIGNAL
#if CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_OPEN
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WEP
#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WPA
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WPA2
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif /*CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD*/

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_FREQ_HZ 100000
double lux_als, lux_white, fc_als, fc_white;


/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_18

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define TIME_ZONE (+8)   //Singapore Time

#define I2C_MASTER_SCL_IO GPIO_NUM_22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_21               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

static const char *TAG1 = "MAIN";

static const char *TAG2 = "scan";
int flag_start=1;
static const char *CONFIG_BROKER_URL="11";
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG2, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		flag_start=0;
    }
}


/* Initialize Wi-Fi as sta and set scan method */
static void fast_scan(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize and start WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD,
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/**
* @brief i2c master initialization
*/
static void i2c_master_init()
{
   i2c_port_t i2c_master_port = I2C_MASTER_NUM;
   i2c_config_t conf;
   conf.mode = I2C_MODE_MASTER;
   conf.sda_io_num = (gpio_num_t) I2C_MASTER_SDA_IO;
   conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
   conf.scl_io_num = (gpio_num_t) I2C_MASTER_SCL_IO;
   conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
   conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
   i2c_param_config( i2c_master_port, &conf);
   if ( i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0) != ESP_OK)
   {
       ESP_LOGE(TAG1, "Failed to initialize I2C driver!\r\n");
   }
}
static const char *TAG = "MQTT_EXAMPLE";


static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
esp_mqtt_client_handle_t client = event->client; int msg_id;
// your_context_t *context = event->context;
switch (event->event_id) {
case MQTT_EVENT_CONNECTED:
ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
break;
case MQTT_EVENT_DISCONNECTED:
ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
break;

case MQTT_EVENT_SUBSCRIBED:
ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
break;
case MQTT_EVENT_UNSUBSCRIBED:
ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
break;
case MQTT_EVENT_PUBLISHED:
ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
break;
case MQTT_EVENT_DATA:
ESP_LOGI(TAG, "MQTT_EVENT_DATA");
printf("TOPIC=%.*s\r\n", event->topic_len, event->topic); printf("DATA=%.*s\r\n", event->data_len, event->data); break;
case MQTT_EVENT_ERROR:
ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
break; default:
ESP_LOGI(TAG, "Other event id:%d", event->event_id);
break;
}
return ESP_OK;
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id); mqtt_event_handler_cb(event_data);
}
esp_mqtt_client_handle_t client;
static void mqtt_app_start(void)
{
esp_mqtt_client_config_t mqtt_cfg = {
.uri = "mqtt://demo.thingsboard.io",
.event_handle = mqtt_event_handler,
.port = 1883,
.username = "yMweV965Uq62zYzNUeNY",

};

 client = esp_mqtt_client_init(&mqtt_cfg); esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client); esp_mqtt_client_start(client);


cJSON *root = cJSON_CreateObject();
cJSON_AddNumberToObject(root, "counter", lux_als);
char *post_data = cJSON_PrintUnformatted(root);
esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0);
cJSON_Delete(root);
// Free is intentional, it's client responsibility to free the result of cJSON_Print free(post_data);
}

void i2c_master_setup(void);
void task_veml7700_read(void *ignore);
void app_main(void)
{
	
	 // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    fast_scan();
	/*ESP_ERROR_CHECK(nvs_flash_init());*/
	ESP_ERROR_CHECK(esp_netif_init());
	//ESP_ERROR_CHECK(esp_event_loop_create_default());
	while(flag_start);
	i2c_master_setup();
	i2c_master_init();
	ESP_LOGI(TAG, "[APP] Startup..");
	ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
	ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

	esp_log_level_set("*", ESP_LOG_INFO);
	esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
	esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
	esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
	mqtt_app_start();
	 printf("Hello from app_main!\n");
	 if ( hdc1080_init(I2C_MASTER_NUM) != ESP_OK)
	 {
		 ESP_LOGE(TAG1, "Failed to initialize HDC1080 sensor!\r\n");
	 }
	 xTaskCreate(&task_veml7700_read, "task_read_veml7700",  4096, NULL, 6, NULL);
}
void i2c_master_setup(void)
{
	i2c_config_t conf;

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = 0;

	i2c_param_config(I2C_MASTER_NUM, &conf);
	i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}
void task_veml7700_read(void *ignore)
{
	float temperature, humidity;
	gpio_pad_select_gpio(BLINK_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	veml7700_handle_t veml7700_dev;

	esp_err_t init_result = veml7700_initialize(&veml7700_dev, I2C_MASTER_NUM);
	if (init_result != ESP_OK) {
		ESP_LOGE("VEML7700", "Failed to initialize. Result: %d\n", init_result);
		return;
	}

	ESP_LOGI("VEML7700", "Reading data...\r\n");

	while (true) {

		 if ( hdc1080_read_temperature(I2C_MASTER_NUM, &temperature, &humidity) == ESP_OK)
		{
			printf("Current temperature = %.2f C, Relative Humidity = %.2f %%\n",
					temperature, humidity);
		}
		// Read the ALS data
		ESP_ERROR_CHECK( veml7700_read_als_lux_auto(veml7700_dev, &lux_als) );
		// Convert to foot candles
		fc_als = lux_als * LUX_FC_COEFFICIENT;
		
		// Read the White data
		ESP_ERROR_CHECK( veml7700_read_white_lux_auto(veml7700_dev, &lux_white) );
		// Convert to foot candles
		// Convert to foot candles
		fc_white = lux_white * LUX_FC_COEFFICIENT;
		cJSON *root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "fc_white", fc_white);
		cJSON_AddNumberToObject(root, "temp", temperature);
		cJSON_AddNumberToObject(root, "humidity", humidity);
		char *post_data = cJSON_PrintUnformatted(root);
		esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0);
		cJSON_Delete(root);
		printf("VEML7700 measured ALS %0.4f lux or %0.4f fc \n", lux_als, fc_als);
		printf("VEML7700 measured White %0.4f lux or %0.4f fc \n\n", lux_white, fc_white);
        
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}
