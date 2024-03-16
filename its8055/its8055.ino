/*
*******************************************************************************
* Copyright (c) 2023 by M5Stack
*                  Equipped with M5StickC sample source code
*                          配套  M5StickC 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/m5stickc
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/m5stickc
*
* Describe: Micophone.
* Date: 2022/2/22
*******************************************************************************
*/
#include <M5StickC.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define PIN_CLK 0
#define PIN_DATA 34
#define READ_LEN (2 * 256)
#define GAIN_FACTOR 3
uint8_t BUFFER[READ_LEN] = { 0 };

uint16_t oldy[160];
int16_t *adcBuffer = NULL;

uint32_t startTime = 0;
bool isRecording = false;

// Set these values according to your configuration.
const char *ssid = "Kein Netz";
const char *password = "kF6)24+hL";
const char *serverIP = "192.168.125.128";
const int serverPort = 1234;

void i2sInit()  // Init I2S.  初始化I2S
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),  // Set the I2S operating mode.
                                                                         // 设置I2S工作模式
    .sample_rate = 44100,                                                // Set the I2S sampling rate.  设置I2S采样率
    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_16BIT,  // Fixed 12-bit stereo MSB.
                                  // 固定为12位立体声MSB
    .channel_format =
      I2S_CHANNEL_FMT_ALL_RIGHT,  // Set the channel format.  设置频道格式
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
    .communication_format =
      I2S_COMM_FORMAT_STAND_I2S,  // Set the format of the communication.
#else                             // 设置通讯格式
    .communication_format = I2S_COMM_FORMAT_I2S,
#endif
    .intr_alloc_flags =
      ESP_INTR_FLAG_LEVEL1,  // Set the interrupt flag.  设置中断的标志
    .dma_buf_count = 2,      // DMA buffer count.  DMA缓冲区计数
    .dma_buf_len = 128,      // DMA buffer length.  DMA缓冲区长度
  };

  i2s_pin_config_t pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
  pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

  pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num = PIN_CLK;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = PIN_DATA;

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void showSignal() {
  int y;
  for (int n = 0; n < 160; n++) {
    y = adcBuffer[n] * GAIN_FACTOR;
    y = map(y, INT16_MIN, INT16_MAX, 10, 70);
    M5.Lcd.drawPixel(n, oldy[n], WHITE);
    M5.Lcd.drawPixel(n, y, BLACK);
    oldy[n] = y;
  }
}

void mic_record_task(void *arg) {
  size_t bytesread;
  while (1) {
    i2s_read(I2S_NUM_0, (char *)BUFFER, READ_LEN, &bytesread,
             (100 / portTICK_RATE_MS));
    adcBuffer = (int16_t *)BUFFER;
    showSignal();
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}
/* After M5StickC is started or reset
  the program in the setUp () function will be run, and this part will only be
  run once. 在 M5StickC
  启动或者复位后，即会开始执行setup()函数中的程序，该部分只会执行一次。 */
void setup() {
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK, WHITE);

  WiFi.begin(ssid, password);  // Connect wifi and return connection status.
                               // 连接wifi并返回连接状态
  M5.lcd.print("Waiting Wifi Connect");
  while (WiFi.status() != WL_CONNECTED) {  // If the wifi connection fails.  若wifi未连接成功
    delay(1000);
    M5.lcd.print(".");
  }
  M5.lcd.println("\nWiFi Connected!");
  M5.lcd.print("WiFi Connect To: ");
  M5.lcd.println(WiFi.SSID());  // Output Network name.  输出网络名称
  M5.lcd.print("IP address: ");
  M5.lcd.println(WiFi.localIP());  // Output IP Address.  输出IP地址

  i2sInit();
  xTaskCreate(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL);
}
/* After the program in setup() runs, it runs the program in loop()
The loop() function is an infinite loop in which the program runs repeatedly
在setup()函数中的程序执行完后，会接着执行loop()函数中的程序
loop()函数是一个死循环，其中的程序会不断的重复运行 */
void loop() {
  static uint32_t startTime = 0;
  static bool isRecording = false;
  static WiFiClient client;

  M5.lcd.setCursor(0, 40);

  // Connect to the server only once
  if (!client.connected()) {
    M5.lcd.printf("Connecting to: %s\n", serverIP);
    if (!client.connect(serverIP, serverPort)) {
      M5.lcd.print("Connection failed.\nWaiting 5 seconds before retrying...\n");
      delay(5000);
      return;
    }
    M5.lcd.println("Connected to server");
  }

  if (!isRecording) {
    startTime = millis();
    isRecording = true;
  }

  if (millis() - startTime <= 2 * 60 * 1000) {
    M5.lcd.print("Recording...\n");
    vTaskDelay(1000 / portTICK_RATE_MS);

    // Send the recorded sound data over the network
    if (client.connected()) {
      client.write((const uint8_t *)BUFFER, READ_LEN);
    }
  } else {
    M5.lcd.print("Recording finished.\n");
    isRecording = false;
    client.stop();

    M5.lcd.println("Waiting 5 seconds before restarting...");
    delay(5000);
    M5.lcd.fillRect(0, 40, 320, 220, BLACK);
  }
}