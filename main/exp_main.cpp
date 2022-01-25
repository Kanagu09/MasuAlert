/* Active click feedback Example */
#include "driver/i2c.h"
#include <driver/adc.h>
#include <driver/mcpwm.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_spi_flash.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <math.h>
#include <nvs_flash.h>
#include <rom/uart.h>
#include <stdio.h>

#include <esp_event_loop.h>
#include <esp_wifi.h>
#include <lwip/dns.h>
#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <cJSON.h>

#include "library/bme280.h"
// #include "library/_i2c.h"
#include "library/I2Cbus.hpp"

const char *TAG = "main";

//----------------------------------------------------------------

#define MAX_COUNT 1000

int ad_w = 0;

int count = 0;

double freq_value = 200.0;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

//----------------------------------------------------------------
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
// #define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a
 * request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each evetime_t,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

// need to rewrite for WiFi AP
// #define WIFI_SSID "ICTEX51"
// #define WIFI_PASS "espwroom32"
#define WIFI_SSID "ICT_EX5_1"
#define WIFI_PASS "embedded"
// **** need to rewrite for your RPi ****
// RPi Flask
#define WEB_SERVER "10.0.0.75"
#define WEB_URL "http://10.0.0.75:50000/getadc?"
#define WEB_PORT "50000"
// **** need to rewrite for your RPi ****
//
// PC -flask
// #define WEB_SERVER "172.16.11.161"
// #define WEB_PORT "5000"
// #define WEB_URL "http://172.16.11.161:5000/getadc?"

//----------------------------------------------------------------
// WiFi Initialize

static esp_err_t event_handler(void *ctx, system_event_t *event) {
    //    ESP_LOGI(TAG, "event_handler.");
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        //        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_START");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        //        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_GOT_IP");
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        //        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_DISCONNECTED");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void) {
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
#if 0
/* start static IP addr */
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);

    tcpip_adapter_ip_info_t ipInfo;
	int sekiji = 0;
    IP4_ADDR(&ipInfo.ip, 172,16,11,70+sekiji);
    IP4_ADDR(&ipInfo.gw, 172,16,11,251);
    IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);

    ip_addr_t dnsserver;
    IP_ADDR4( &dnsserver, 172,16,11,251);
    dns_setserver(0, &dnsserver);
/* end static IP addr */
#endif
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    /* ------------------------*/
    //    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", WIFI_SSID);
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    sprintf(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID);
    sprintf(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASS);
    //    wifi_config_t wifi_config = { };
    //    wifi_config.sta.ssid=(char *) WIFI_SSID;
    //    wifi_config.sta.password=(char *) WIFI_PASS;
    /* ------------------------*/
    //    wifi_config_t wifi_config = {
    //        .sta = {
    //            { .ssid = WIFI_SSID },
    //            { .password = WIFI_PASS },
    ////            { .ssid = EXAMPLE_WIFI_SSID },
    ////            { .password = EXAMPLE_WIFI_PASS },
    //        },
    //    };
    /* ------------------------*/
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...",
             wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(
        esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

//----------------------------------------------------------------
// Data Send and receive

static void http_get_task(void *pvParameters) {

    //    const struct addrinfo hints = {
    //        { .ai_family = AF_INET },
    //        { .ai_socktype = SOCK_STREAM }
    //    };
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[128];

    while(1) {
        const char *REQUEST1 = "GET " WEB_URL;
        const char *REQUEST2 = " HTTP/1.0\r\n"
                               "Host: " WEB_SERVER "\r\n"
                               "User-Agent: esp-idf/1.0 esp32\r\n"
                               "\r\n";
        char REQUEST[2048];

        ESP_LOGI(TAG, "restart http_get_task : ADC %d count %d", ad_w, count);

        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
        //        ESP_LOGI(TAG, "Start Connection to AP: ADC %d", ad);
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true,
                            portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real"
           code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        //  get adc value from global var.

        strcpy(REQUEST, REQUEST1);
        sprintf(REQUEST + strlen(REQUEST), "ADC=%d", ad_w);
        strcat(REQUEST, REQUEST2);
        if(write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if(setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                      sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        char buf[1024];
        buf[0] = '\0';
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf) - 1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
            strcat(buf, recv_buf);
        } while(r > 0);
        /* ------------------------*/
        // printf("Buffer %s\n", buf);
        cJSON *root = NULL;
        char *bufptr;

        for(bufptr = buf; *bufptr != '\0' && *bufptr != '{'; bufptr++)
            ;
        if(*bufptr == '{') {
            //   printf("Buffer stripped %s\n", bufptr);
            root = cJSON_Parse(bufptr);
            if(root == NULL) {
                ESP_LOGE(TAG, "... received wrong json format");
                continue;
            }
            if(cJSON_GetObjectItem(root, "freq") == NULL) {
                ESP_LOGE(TAG, "... freq not found");
                continue;
            }
            //    printf("freq: %s\n", cJSON_Print(cJSON_GetObjectItem(root,
            //    "freq")));
            freq_value = atof(cJSON_Print(cJSON_GetObjectItem(root, "freq")));
            printf("received freq_value = %lf\n", freq_value);
            /* check the received freq_value */
            if(freq_value < 100)
                freq_value = 100;
            else if(freq_value > 500)
                freq_value = 500;
            printf(" modified freq_value = %lf\n", freq_value);
        }

        ESP_LOGI(
            TAG,
            "... done reading from socket. Last read return=%d errno=%d\r\n", r,
            errno);
        close(s);
        vTaskDelay(1000 / portTICK_PERIOD_MS); /* wait for 1000 ms */
    }                                          /* end while(1) */
}

//----------------------------------------------------------------
// #define USE_TIMER   //  Whther use the timer or not. Without this definition,
// the function is called from a normal task.

#ifdef USE_TIMER
#define DT                                                                     \
    0.0001 //  In the case of the timer, the minimum period is 50 micro second.
#else
#define DT (1.0 / configTICK_RATE_HZ)
//  In the case of the task, the time period is the time slice of the OS
//  specified in menuconfig, which is set to 1 ms=1 kHz.
#endif

struct WaveParam {
    const double damp[3] = {-10, -20, -30};
    const int nDamp = sizeof(damp) / sizeof(damp[0]);
    const double freq[4] = {100, 200, 300, 500};
    const int nFreq = sizeof(freq) / sizeof(freq[0]);
    const double amplitude = 2;
} wave; //

double time_c = -1;

signed long int t_fine;

signed long int calibration_T(signed long int adc_T) {

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) *
            ((signed long int)dig_T2)) >>
           11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) *
              ((adc_T >> 4) - ((signed long int)dig_T1))) >>
             12) *
            ((signed long int)dig_T3)) >>
           14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

unsigned long int calibration_H(signed long int adc_H) {
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) -
               (((signed long int)dig_H5) * v_x1)) +
              ((signed long int)16384)) >>
             15) *
            (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                 (((v_x1 * ((signed long int)dig_H3)) >> 11) +
                  ((signed long int)32768))) >>
                10) +
               ((signed long int)2097152)) *
                  ((signed long int)dig_H2) +
              8192) >>
             14));
    v_x1 =
        (v_x1 -
         (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >>
          4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (unsigned long int)(v_x1 >> 12);
}

void hapticFunc(void *arg) {
    const char *TAG = "H_FUNC";
    static int i;            //  An integer to select waveform.
    static double omega = 0; //  angular frequency
    static double B = 0;     //  damping coefficient
    int ad = 0;

    /* -------- */
    uint8_t osrs_t = 1;   // Temperature oversampling x 1
    uint8_t osrs_p = 0;   // Pressure oversampling x 1
    uint8_t osrs_h = 1;   // Humidity oversampling x 1
    uint8_t mode = 3;     // Normal mode
    uint8_t t_sb = 5;     // Tstandby 1000ms
    uint8_t filter = 0;   // Filter off
    uint8_t spi3w_en = 0; // 3-wire SPI Disable

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_meas_regctrl_hum_reg = osrs_h;

    I2C_t &myI2C = i2c0;
    myI2C.begin(GPIO_NUM_25, GPIO_NUM_21);

    myI2C.setTimeout(10);
    myI2C.scanner();

    myI2C.writeByte(0x76, 0xF2, ctrl_meas_regctrl_hum_reg);
    myI2C.writeByte(0x76, 0xF4, ctrl_meas_reg);
    myI2C.writeByte(0x76, 0xF5, config_reg);

    // mpu9250_register_write_byte(0xF2, ctrl_meas_reg);

    /* -------- */
    // read
    uint8_t raw[8];
    for(int i = 0; i < 8; i++) {
        myI2C.readByte(0x76, 0xF7 + i, &raw[i]);
    }

    unsigned long int tmp_raw = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);
    unsigned long int hum_raw = (raw[6] << 8) | raw[7];
    printf("%d, %d, %d\n", raw[3], raw[4], raw[5]);
    printf("%d, %d\n", raw[6], raw[7]);

    // calibration
    uint8_t data[33];
    for(int i = 0; i < 24; i++) {
        // printf("%d -> ", data[i]);
        myI2C.readByte(0x76, 0x88 + i, &data[i]);
        // printf("%d\n", data[i]);
    }
    myI2C.readByte(0x76, 0xA1, &data[24]);

    for(int i = 0; i < 8; i++) {
        // printf("%d -> ", data[25 + i]);
        myI2C.readByte(0x76, 0xE1 + i, &data[25 + i]);
        // printf("%d\n", data[25 + i]);
    }

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26] << 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28] << 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];

    signed long int tmp = calibration_T(tmp_raw);
    signed long int hum = calibration_H(hum_raw);

    printf("tmp = %f\n", (double)tmp / 100);
    printf("hum = %f\n", (double)hum / 1024);

    ad_w = ad;

    /* -------- */
    if(ad < 2100 && time_c > 0.3) {
        time_c = -1;
        printf("\r\n");
    }
    if(ad > 2400 && time_c == -1) { //  When the button is pushed after
                                    //  finishing to output an wave.
        //  set the time_c to 0 and update the waveform parameters.
        time_c = 0;
        // Frequency
        omega = freq_value * M_PI * 2;
        // omega = wave.freq[i % wave.nFreq] * M_PI * 2;
        B = wave.damp[i / wave.nFreq];
        // printf("Wave: %3.1fHz, A=%2.2f, B=%3.1f ", omega / (M_PI * 2),
        //        wave.amplitude, B);
        i++;
        if(i >= wave.nFreq * wave.nDamp)
            i = 0;
    }
    //  Output the wave
    double pwm = 0;
    if(time_c >= 0) {
        pwm = wave.amplitude * cos(omega * time_c) * exp(B * time_c);
        time_c += DT;
    } else {
        pwm = 0;
    }

    //  Set duty rate of pwm
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm * 100);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    count++;
    if(count >= 1000) {
        ESP_LOGI(TAG, "count 1000 : ADC %d count %d", ad, count);
        count = 0;
    }
}

#ifndef USE_TIMER
void hapticTask(void *arg) {
    while(1) {
        hapticFunc(arg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif

extern "C" void app_main()
// void app_main()
{
    //----------------------------------
    // Initialize NVS
    ESP_LOGI("main", "Initialize NVS");
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //----------------------------------

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                         : "external");

    //----------------------------------
    ESP_LOGI("main", "Initialize WiFi");
    initialise_wifi();
    //----------------------------------
    // i2c_master_init();

    //----------------------------------
    printf("!!! Active Haptic Feedback Start !!!\n");

    ESP_LOGI("main", "Initialize ADC");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    ESP_LOGI("main", "Initialize PWM");
    // 1. mcpwm gpio initialization
    const int GPIO_PWM0A_OUT = 16;
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    // 2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // frequency = 1000Hz,
    pwm_config.cmpr_a = 0;       // duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,
               &pwm_config); // Configure PWM0A with above settings
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 20000);
    gpio_config_t conf;
    conf.pin_bit_mask = (1 << (17)) | (1 << (5));
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);

#ifdef USE_TIMER
    esp_timer_init();
    esp_timer_create_args_t timerDesc = {
        callback : hapticFunc,
        arg : NULL,
        dispatch_method : ESP_TIMER_TASK,
        name : "haptic"
    };
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&timerDesc, &timerHandle);
    esp_timer_start_periodic(
        timerHandle,
        (int)(1000 * 1000 * DT)); // period in micro second (100uS=10kHz)
#else
    TaskHandle_t taskHandle = NULL;
    xTaskCreate(hapticTask, "Haptic", 1024 * 15, NULL, 6, &taskHandle);
#endif
    xTaskCreate(&http_get_task, "http_get_task", 1024 * 15, NULL, 3, NULL);

    uart_driver_install(UART_NUM_0, 1024, 1024, 10, NULL, 0);
    while(1) {
        uint8_t ch;
        uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        printf("'%c' received.\r\n", ch);
        switch(ch) {
        case 'a':
            //  do something
            break;
        }
    }
}