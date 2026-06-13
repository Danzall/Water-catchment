/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "mqtt_client.h"
#include <time.h>
#include "esp_sntp.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_sntp.h" // For NTP synchronization
//#include "esp_time.h" // For time functions (gettimeofday)
#include <time.h>     // For tm structure

#include <esp_event.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"

#include "esp_netif_sntp.h"
#include <sys/time.h>
#include "nvs_flash.h"

#include "driver/mcpwm_cap.h"
#include "esp_private/esp_clk.h"

static const char *TAG = "water catchment";



#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100

#define EXAMPLE_EC11_GPIO_A 0
#define EXAMPLE_EC11_GPIO_B 2

#define DEFAULT_SSID CONFIG_EXAMPLE_WIFI_SSID
#define DEFAULT_PWD CONFIG_EXAMPLE_WIFI_PASSWORD

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
#if CONFIG_SOC_WIFI_SUPPORT_5G || CONFIG_SLAVE_SOC_WIFI_SUPPORT_5G
#define DEFAULT_RSSI_5G_ADJUSTMENT CONFIG_EXAMPLE_FAST_SCAN_RSSI_5G_ADJUSTMENT
#else
#define DEFAULT_RSSI_5G_ADJUSTMENT 0
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#define DEFAULT_RSSI_5G_ADJUSTMENT 0
#endif /*CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD*/

#define MOTORAC1 14
#define MOTORAC2 27
#define MOTORDC 25
#define TANK1 21
#define TANK2 22
#define TANK3 33
#define BUZZER 16
#define LED 2

#define HC_SR04_TRIG_GPIO  12
#define HC_SR04_ECHO_GPIO  27

#define HC_SR04_TRIG_GPIO_1 26
#define HC_SR04_ECHO_GPIO_1  25

EventGroupHandle_t s_wifi_event_group = NULL;

// --- RTC RAM Variable ---
RTC_DATA_ATTR int boot_count = 0; // Persists across reboots & sleep
esp_mqtt_client_handle_t client;

typedef struct sensor {
    uint32_t level;
    uint8_t state;
    uint8_t error;
    char errorMessage[30];
}
sensor;

sensor sensor1;
sensor sensor2;
sensor sensor3;


uint8_t lightStatus = 0;
uint8_t wifiStatus = 0;
uint32_t motionTime = 0;
uint32_t sensorPower = 0;
uint32_t sensorTimer = 0;
uint32_t senor1Time = 0;
uint32_t senor2Time = 0;
uint32_t senor3Time = 0;
uint32_t senor4Time = 0;
uint32_t presenceTimer = 0;

typedef enum state{
    idle,
    powerOn,
    sense,
    single
} state;

state alarmState = idle;
float sense1;
float sense2;
float sense3;

void sntpInit();

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void process(char* topic, uint8_t topicLen, char* msg, uint8_t msgLen){
    //ESP_LOGI(TAG, topic);
    printf("topic=%.*s\r\n", topicLen,topic);
    if(strncmp("/request",topic,topicLen) == 0){
        ESP_LOGI(TAG, "got request");
    }
    if(strncmp("motor/controlAC1",topic, topicLen) == 0){
        ESP_LOGI(TAG, "set motor control");
        if(strncmp("ON",msg, msgLen) == 0){
            alarmState = powerOn;
            gpio_set_level(MOTORAC1, 1);
            esp_mqtt_client_publish(client, "motor/response", "AC1 pump on", 0, 1, 0);
        }
        if(strncmp("OFF",msg, msgLen) == 0){                                                       
            //gpio_set_level(SENSORPWR, 0);
            gpio_set_level(MOTORAC1, 0);
            esp_mqtt_client_publish(client, "motor/response", "AC1 pump off", 0, 1, 0);
        }
        if(strncmp("STATUS",msg, msgLen) == 0){
            char temp[30];
            //sprintf(temp,"%i,%li,%li,%li,%li",alarmState, sensor1.time,sensor2.time,sensor3.time,sensor4.time);
            esp_mqtt_client_publish(client, "motor/response", temp, 0, 1, 0);
        }
    }
    if(strncmp("motor/controlAC2",topic, topicLen) == 0){
        ESP_LOGI(TAG, "set motor control");
        if(strncmp("ON",msg, msgLen) == 0){
            alarmState = powerOn;
            esp_mqtt_client_publish(client, "motor/response", "AC2 pump on", 0, 1, 0);
        }
        if(strncmp("OFF",msg, msgLen) == 0){
            //gpio_set_level(SENSORPWR, 0);
            esp_mqtt_client_publish(client, "motor/response", "AC2 pump off", 0, 1, 0);
        }
        if(strncmp("STATUS",msg, msgLen) == 0){
            char temp[30];
            //sprintf(temp,"%i,%li,%li,%li,%li",alarmState, sensor1.time,sensor2.time,sensor3.time,sensor4.time);
            esp_mqtt_client_publish(client, "motor/response", temp, 0, 1, 0);
        }
    }
    if(strncmp("motor/controlDC",topic, topicLen) == 0){
        ESP_LOGI(TAG, "set motor control");
        if(strncmp("ON",msg, msgLen) == 0){
            alarmState = powerOn;
            esp_mqtt_client_publish(client, "motor/response", "DC pump on", 0, 1, 0);
        }
        if(strncmp("OFF",msg, msgLen) == 0){
            //gpio_set_level(SENSORPWR, 0);
            esp_mqtt_client_publish(client, "motor/response", "DC pump off", 0, 1, 0);
        }
        if(strncmp("STATUS",msg, msgLen) == 0){
            char temp[30];
            //sprintf(temp,"%i,%li,%li,%li,%li",alarmState, sensor1.time,sensor2.time,sensor3.time,sensor4.time);
            esp_mqtt_client_publish(client, "motor/response", temp, 0, 1, 0);
        }
    }
    if(strncmp("threshold/set",topic, topicLen) == 0){
        /*handle threshold set event*/
        
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "conn/update", "MQTT connected", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "motor/controlAC1", 0);
        ESP_LOGI(TAG, "sent AC1 subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "motor/controlAC2", 0);
        ESP_LOGI(TAG, "sent AC2 subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "motor/controlDC", 0);
        ESP_LOGI(TAG, "sent DC subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "threshold/set", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        gpio_set_level(LED, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        wifiStatus = 1;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "sub/update", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        process(event->topic, event->topic_len, event->data, event->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        //.broker.address.uri = CONFIG_BROKER_URL,
        .broker.address.uri = "mqtt://gsmdev.ddns.net",
        //.broker.address.uri = "mqtt://smartsensor.ddns.net",
        //.credentials.username = "DES370S",
        //.credentials.authentication.password = "IoTwaterMeter",
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    /*Let's enqueue a few messages to the outbox to see the allocations*/
    int msg_id;
    msg_id = esp_mqtt_client_enqueue(client, "catchment/register", "data_que", 0, 1, 0, true);
    ESP_LOGI(TAG, "Enqueued msg_id=%d", msg_id);
    msg_id = esp_mqtt_client_enqueue(client, "catchment/qos2", "QoS2 message", 0, 2, 0, true);
    ESP_LOGI(TAG, "Enqueued msg_id=%d", msg_id);

    /* Now we start the client and it's possible to see the memory usage for the operations in the outbox. */
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "Connect MQTT");
}

void sensorPublish(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting random tank level task");

    while (1) {
        int min = 1;
        int max = 100;

        int random_number = rand() % (max - min + 1) + min;

        char temp[20];
        sprintf(temp,"%.2f",sense1);
        esp_mqtt_client_publish(client, "tank/main", (const char*)temp, 0, 1, 0);

        /*printf("Random number1 between %d and %d: %d\n", min, max, random_number);
        
        sprintf(temp,"%i",random_number);
        esp_mqtt_client_publish(client, "tank/main", (const char*)temp, 0, 1, 0);
        random_number = rand() % (max - min + 1) + min;

        printf("Random number1 between %d and %d: %d\n", min, max, random_number);
        
        sprintf(temp,"%i",random_number);
        esp_mqtt_client_publish(client, "tank/reservoir", (const char*)temp, 0, 1, 0);
        random_number = rand() % (max - min + 1) + min;

        printf("Random number1 between %d and %d: %d\n", min, max, random_number);
        
        sprintf(temp,"%i",random_number);
        esp_mqtt_client_publish(client, "tank/hydroponic", (const char*)temp, 0, 1, 0);*/

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "Running OTA example task");
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        mqtt_app_start();
        wifiStatus = 1;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
static void fast_scan(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

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
            //.ssid = "Openserve-1D19",
            //.password = "XYHD66eq44",
            //.ssid = "TP-Link_5E",
            //.password = "Danzall123",
            //.ssid = "CFT_2.4G",
            //.password = "underadome",
            .ssid = "SEG260",
            .password = "test1234",
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
            .threshold.rssi_5g_adjustment = DEFAULT_RSSI_5G_ADJUSTMENT,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void counterTask(){
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // Report counter value
    int pulse_count = 0;
    int event_count = 0;
    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
            ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        } else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        }
    }
}

static void configure_io(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED);
    
    /* Set the GPIO as a push/p
    ull output */
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTORAC1, GPIO_MODE_OUTPUT);
    /*gpio_set_direction(MOTORAC2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTORDC, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_direction(TANK1, GPIO_MODE_INPUT);
    gpio_set_direction(TANK2, GPIO_MODE_INPUT);
    gpio_set_direction(TANK3, GPIO_MODE_INPUT);
    gpio_set_direction(33, GPIO_MODE_INPUT);

    gpio_set_pull_mode(TANK1, GPIO_PULLUP_PULLDOWN);
    gpio_set_pull_mode(TANK2, GPIO_PULLUP_PULLDOWN);
    gpio_set_pull_mode(TANK3, GPIO_PULLUP_PULLDOWN);*/
    //gpio_set_pull_mode(TANK1, GPIO_PULLUP_PULLDOWN);

    /*gpio_reset_pin(SENSOR1);
    gpio_reset_pin(SENSOR2);
    gpio_reset_pin(SENSOR3);
    gpio_reset_pin(SENSOR4);*/
    gpio_set_level(LED, 0);
}

uint8_t getSensor(uint8_t index){
    sensor* temp;
    //if(index == SENSOR1) temp = &sensor1;

    if(gpio_get_level(index) == 1){
        gpio_set_level(LED, 1);
        //esp_mqtt_client_publish(client, "/willow/sen1", "motion detected", 0, 1, 0);
        ESP_LOGI(TAG, "motion %i", index);
        return 1;
    }
    else if(!gpio_get_level(index)){
        gpio_set_level(LED, 0);
        ESP_LOGI(TAG, "stop");
    }
    return 0;
}

void readSensor(){
    while(1){
        switch(alarmState){
            case idle:
            
            break;
            case powerOn:
                presenceTimer++;
                ESP_LOGI(TAG, "Sensor detect");
                //if(gpio_get_level(SENSOR1)) sensor1.presence = 1;
                
            break;
            case sense:
                //ESP_LOGI(TAG, "Read sensors");
                //if(getSensor(SENSOR1)) esp_mqtt_client_publish(client, "/willow/sen1", "motion 1 detected", 0, 1, 0);

            break;
            case single:
                
                break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}



static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

/**
 * @brief generate single pulse on Trig pin to start a new sample
 */
static void gen_trig_output(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 1); // set high
    gpio_set_level(HC_SR04_TRIG_GPIO_1, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0); // set low
    gpio_set_level(HC_SR04_TRIG_GPIO_1, 0); // set low
}

void readSensor1(){

    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks;
    while(1){
        // trigger the sensor to start a new sample
        gen_trig_output();
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000) {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float) pulse_width_us / 58;
            sense1 = distance;
            ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void readSensor2(){

    ESP_LOGI(TAG, "Install capture timer2");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO_1,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks;
    while(1){
        // trigger the sensor to start a new sample
        gen_trig_output();
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000) {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float) pulse_width_us / 58;
            sense1 = distance;
            ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void captureInit(){
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
}



void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    configure_io();
    fast_scan();
    //captureInit();
    xTaskCreate(sensorPublish, "publish_task", 8192, NULL, 5, NULL);
    //xTaskCreate(&counterTask, "counter_task", 8192, NULL, 5, NULL);
    xTaskCreate(readSensor1, "level_task", 8192, NULL, 5, NULL);
    xTaskCreate(readSensor2, "level_task2", 8192, NULL, 5, NULL);
}
