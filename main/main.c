#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/event_groups.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "input_iot.h"
#include "output_iot.h"
#include "driver/i2c.h"
#include "uart.h"

#include "esp_system.h"
#include "esp_wifi.h"   
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define MPU6050_ADDR 0x68 // Địa chỉ I2C của MPU6050
#define BLINK_GPIO 2
static uint8_t isPress = 0;

void delay(uint32_t time) {
    vTaskDelay(time / portTICK_PERIOD_MS);
}


void funcionButton(void *arg)
{
    while(1) {
        if(gpio_get_level(GPIO_NUM_0) ==  0) {
            delay(20);
            if((gpio_get_level(GPIO_NUM_0) == 0) && (isPress == 0)) {
                    call_SIM800l(NULL);
                    printf("gọi\n");
                    isPress = 1;
            }
        } else {
            isPress = 0;
        }
    }
}



void button_timeout_callback(int pin)
{
    if(pin ==  GPIO_NUM_0)
    {
        printf("TIMEOUT\n");
        // checkConnectSIM800l(NULL);
       //call_SIM800l(NULL);
        gpio_set_level(BLINK_GPIO, 1);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
    }
}




void mpu6050_task(void *pvParameter)
{
    
    i2c_cmd_handle_t cmd;
    uint8_t data_raw[14]; // Dữ liệu raw của MPU6050 (14 byte)
    int16_t ax, ay, az, gx, gy, gz; // Giá trị Acceleration và Rotation
    float Ax = 0,Ay = 0,Az = 0, Gx = 0, Gy = 0,Gz = 0;
    bool fall = false; //stores if a fall has occurred
    bool trigger1=false; //stores if first trigger (lower threshold) has occurred
    bool trigger2=false; //stores if second trigger (upper threshold) has occurred
    bool trigger3=false; //stores if third trigger (orientation change) has occurred
    uint8_t trigger1count=0; //stores the counts past since trigger 1 was set true
    uint8_t trigger2count=0; //stores the counts past since trigger 2 was set true
    uint8_t trigger3count=0; //stores the counts past since trigger 3 was set true
    int angleChange=0;
    

    // Khởi tạo giao tiếp I2C với MPU6050
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21, // GPIO21 được sử dụng làm SDA
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22, // GPIO22 được sử dụng làm SCL
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000 // Tốc độ truyền dữ liệu 100kHz
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Thiết lập MPU6050
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // Địa chỉ thanh ghi PWR_MGMT_1
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // Thiết lập chế độ đo +-8g
   /* cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1C, true); // Địa chỉ thanh ghi ACCEL_CONFIG
    i2c_master_write_byte(cmd, 0x10, true); // Ghi giá trị 0x10 vào thanh ghi ACCEL_CONFIG để chuyển sang chế độ +- 8g
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);*/

    while(1) {
        // Đọc dữ liệu từ MPU6050
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x3B, true); // Địa chỉ thanh ghi ACCEL_XOUT_H
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data_raw, 14, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
         // Tính giá trị Acceleration và Rotation từ dữ liệu raw
        ax = (data_raw[0] << 8 | data_raw[1]);
        ay = (data_raw[2] << 8 | data_raw[3]);
        az = (data_raw[4] << 8 | data_raw[5]);
        gx = (data_raw[8] << 8 | data_raw[9]);
        gy = (data_raw[10] << 8 | data_raw[11]);
        gz = (data_raw[12] << 8 | data_raw[13]);

    // In giá trị Acceleration và Rotation ra Serial Monitor
        //printf("Acceleration: (%d, %d, %d), Rotation: (%d, %d, %d)\n", ax, ay, az, gx, gy, gz);


        Ax = (ax)/16384.00;
        Ay = (ay)/16384.00;
        Az = (az)/16384.00;
        Gx = (gx)/131.07;
        Gy = (gy)/131.07;
        Gz = (gz)/131.07;

        float Raw_Amp = pow(pow(Ax,2)+pow(Ay,2)+pow(Az,2),0.5);
        int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
       // printf("Amp: %d\n", Amp);
        if (Amp<=2 && trigger2==false){ //if AM breaks lower threshold (0.4g)
        trigger1=true;
        printf("TRIGGER 1 ACTIVATED\n");
        }
        if (trigger1==true){
        trigger1count++;
        if (Amp>=12){ //if AM breaks upper threshold (3g)
            trigger2=true;
            printf("TRIGGER 2 ACTIVATED\n");
            trigger1=false; trigger1count=0;
            }
        }
        if (trigger2==true){
        trigger2count++;
        angleChange = pow(pow(Gx,2)+pow(Gy,2)+pow(Gz,2),0.5);
        printf("AngleChange:%d\n", angleChange);
        if (angleChange>=30 && angleChange<=400){ //if orientation changes by between 80-100 degrees
            trigger3=true; trigger2=false; trigger2count=0;
            printf("AngleChange:%d\n", angleChange);
            printf("TRIGGER 3 ACTIVATED\n");
            }
        }
        if (trigger3==true){
            trigger3count++;
            if (trigger3count>=10){ 
            angleChange = pow(pow(Gx,2)+pow(Gy,2)+pow(Gz,2),0.5);
            //delay(10);
             printf("AngleChange:%d\n", angleChange);
            if ((angleChange>=0) && (angleChange<=50)){ //if orientation changes remains between 0-10 degrees
                fall=true; trigger3=false; trigger3count=0;
                printf("AngleChange:%d\n", angleChange);
                    }
            else{ //user regained normal orientation
                trigger3=false; trigger3count=0;
                printf("TRIGGER 3 ACTIVATED\n");
            }
            }
        }
        if (fall==true){ //in event of a fall detection
        printf("FALL DETECTED\n");
        call_SIM800l(NULL);
        gpio_set_level(BLINK_GPIO, 1);
        fall=false;
        }
        if (trigger2count>=6){ //allow 0.5s for orientation change
        trigger2=false; trigger2count=0;
        printf("TRIGGER 2 DECACTIVATED\n");
        }
        if (trigger1count>=6){ //allow 0.5s for AM to break upper threshold
        trigger1=false; trigger1count=0;
        printf("TRIGGER 1 DECACTIVATED\n");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static const char *TAG = "example"; // chuỗi nhãn để xác định vị trí trong các tin nhắn log 

#define WEB_SERVER "api.thingspeak.com" //tên máy chủ
#define WEB_PORT "80" //cổng máy chủ được yêu cầu
char REQUEST[512];
char SUBREQUEST[100];
char recv_buf[512];


static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = { //kiểu cấu trúc addrinfo với tên là hints, dùng để chỉ ra yêu cầu cho việc phân giải DNS và tạo socket
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    //res và addr để lưu trữ thông tin kết quả phân giải DNS
    struct addrinfo *res;
    struct in_addr *addr;
    //s tương ứng với file descriptor của socket; "r" là biến sẽ được sử dụng để lưu trữ số lượng byte đã nhận từ server khi đọc dữ liệu từ socket
    int s, r;
    

    while(1) {
        //phân giải tên miền webserver
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0); //tạo socket: res -> ai_family: kiểu giao thức mạng IPv6 or v4, res-> ai_socktype: loại socket TCP or UDP
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

        sprintf(SUBREQUEST, "api_key=DO7380ZRBP2LVRDH&field1=%d&field2=%d", 50, 50);
        sprintf(REQUEST, "POST /update HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length:%d\r\n\r\n%s", strlen(SUBREQUEST), SUBREQUEST);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}


void app_main()
{   
    initUart();
    a7670c_setup();
    output_io_create(BLINK_GPIO);
    input_io_create(GPIO_NUM_0, ANY_EDLE, 0);
    input_set_timeout_callback(button_timeout_callback);  
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 0, NULL); 
    xTaskCreate(funcionButton, "button", 2048, NULL, 0, NULL);            

     ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL); 
}