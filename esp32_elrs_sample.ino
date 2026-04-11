#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

//ESP32-C3: 我选择使用2,3,5,6,7,8,10,12作为6个LEDC输出驱动外设. 4作为通讯状态指示灯. IO1用来读取电池电压. 14,15,16,17作为SPI. 第2个串口UART1：IO18 / IO19. USB串口UART0：IO20 / IO21
#define PIN_RX 18 //ESP32_C3暖色
#define PIN_TX 19 //ESP32_C3冷色

//CRSF通讯状态指示
#define PIN_LED 4   //linkStatusLed, ESP32_C3绿色

//定义LEDC通道用于硬件PWM输出. ESP32-C3支持最大6个通道
#define LEDC_OUTPUT_PIN1 2
#define LEDC_OUTPUT_PIN2 5
#define LEDC_OUTPUT_PIN3 6
#define LEDC_OUTPUT_PIN4 7
#define LEDC_OUTPUT_PIN5 8

//ADC
#define ADC_PIN 1  //可用于电池电压

// use 12 bit precision for LEDC timer
#define LEDC_TIMER_BIT 12   //ESP32-C3只支持最大14bit. 	12bit是0 ~ 4095
// LEDC frequency
#define LEDC_FREQ_1 50    
#define LEDC_FREQ_2 50    
#define LEDC_FREQ_3 400   //400Hz 用于电调和数字舵机
#define LEDC_FREQ_4 50
#define LEDC_FREQ_5 50    //50Hz 用于传统舵机,高频率不支持

// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

void setup()
{
  Serial.begin(115200);
  Serial.println("COM Serial initialized");
  
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial);

  pinMode(PIN_LED, OUTPUT);
  analogWrite(PIN_LED, 0);

  analogReadResolution(12);   //set the resolution to 12 bits (0-4095)
  //analogSetAttenuation(ADC_ATTEN_DB_11);  //0 mV ~ 2500 mV

  ledcAttach(LEDC_OUTPUT_PIN1, LEDC_FREQ_1, LEDC_TIMER_BIT);
  ledcAttach(LEDC_OUTPUT_PIN2, LEDC_FREQ_2, LEDC_TIMER_BIT);
  ledcAttach(LEDC_OUTPUT_PIN3, LEDC_FREQ_3, LEDC_TIMER_BIT);
  ledcAttach(LEDC_OUTPUT_PIN4, LEDC_FREQ_4, LEDC_TIMER_BIT);
  ledcAttach(LEDC_OUTPUT_PIN5, LEDC_FREQ_5, LEDC_TIMER_BIT);

}

void loop()
{
    // Must call crsf.update() in loop() to process data
    crsf.update();

    //根据CRSF通讯状态点灯
    updateLinkStatusLed();

    float RxBt = analogRead(ADC_PIN) * 3.3f / 4095.0;


    //将CRSF 1-6通道输出硬件LEDC PWM
    ledcWrite(LEDC_OUTPUT_PIN1, ToDuty(crsf.getChannel(1), 1000, 2000, LEDC_FREQ_1, LEDC_TIMER_BIT, 500, 2500));
    ledcWrite(LEDC_OUTPUT_PIN2, ToDuty(crsf.getChannel(2), 1000, 2000, LEDC_FREQ_2, LEDC_TIMER_BIT, 500, 2500));
    ledcWrite(LEDC_OUTPUT_PIN3, ToDuty(crsf.getChannel(3), 1000, 2000, LEDC_FREQ_3, LEDC_TIMER_BIT, 1000, 2000)); //Thr
    ledcWrite(LEDC_OUTPUT_PIN4, ToDuty(crsf.getChannel(4), 1000, 2000, LEDC_FREQ_4, LEDC_TIMER_BIT, 500, 2500));
    ledcWrite(LEDC_OUTPUT_PIN5, ToDuty(crsf.getChannel(6), 1000, 2000, LEDC_FREQ_5, LEDC_TIMER_BIT, 500, 2500));

    //定期执行
    static uint32_t lastTick = 0;
    if (millis() - lastTick > 500) {
        //
        //打印各通道的值(每个通道范围1000-2000)
        printChannels();
        //回传电池数据
        sendRxBattery(RxBt, 1.1, 11, 11);  //遥控器TELEMETRY中查看RxBt, Curr,Capa,Bat%

        //
        lastTick = millis();
    }

}

//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

static void sendRxBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

void updateLinkStatusLed()
{
  if(crsf.isLinkUp())
  {
    analogWrite(PIN_LED, 80); // 软件PWM, 范围 0~255
  }
  else
  {
    analogWrite(PIN_LED, 0);
    // Perform the failsafe action
  }
}


uint32_t ToDuty(uint16_t Value,
                    uint16_t minValue,
                    uint16_t maxValue,
                    uint32_t freq,
                    uint8_t resolution,
                    uint16_t minPulseUs,
                    uint16_t maxPulseUs)
{
  // Value: 输入值
  // minValue: 输入值的下限
  // maxValue: 输入值上限
  // freq: LEDC频率（比如舵机50Hz,电调400Hz）
  // resolution: LEDC分辨率（比如 12bit, 0 ~ 4095）
  // minPulseUs / maxPulseUs: 舵机/电调支持的脉宽范围（比如舵机500~2500, 电调1000~2000）
  /////////////////////////////////////////////////////////////////
  // 1. 限制输入范围
  if (Value < minValue) Value = minValue;
  if (Value > maxValue) Value = maxValue;

  // 2. 映射 Value → 脉宽（浮点除法）
  float t = float(Value - minValue) / float(maxValue - minValue);   // 0.0 ~ 1.0
  float pulseUs = minPulseUs + t * (maxPulseUs - minPulseUs);

  // 3. 计算 LEDC 周期（微秒）
  float periodUs = 1000000.0f / freq;

  // 4. 计算最大 duty
  uint32_t maxDuty = (1u << resolution) - 1u;

  // 5. 换算成 duty
  uint32_t duty = (pulseUs / periodUs) * maxDuty;

  // 6. 安全限制
  if (duty > maxDuty) duty = maxDuty;

  return duty;
}

