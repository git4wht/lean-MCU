#include <Arduino.h>
#include <TimerInterrupt_Generic.h>

#define TIMER1_INTERVAL_MS 20

// #define REFRESH_INTERVAL 16
// #define CROSSFIRE_BAUD_RATE 115200
// #define REFRESH_INTERVAL 4
// #define CROSSFIRE_BAUD_RATE 400000

// high speed mode working!!!!.
// 刷新间隔
#define REFRESH_INTERVAL 4
// 波特率
#define SBUS_BAUD_RATE 115200
#define CROSSFIRE_BAUD_RATE 400000

// CRSF 每个通道位数
#define CROSSFIRE_CH_BITS 11
// CRSF 中心点
#define CROSSFIRE_CENTER 0x3E0 // 992
// CRSF 最小值
#define CROSSFIRE_LOW 173
// CRSF 最大值
#define CROSSFIRE_HIGH 1815
// CRSF 中心偏移
#define CROSSFIRE_CENTER_CH_OFFSET(ch) (0)
// CRSF 通道数
#define CROSSFIRE_CHANNELS_COUNT 16
// 模型地址
#define MODULE_ADDRESS 0xEE
// 通道ID
#define CHANNELS_ID 0x16
// CRSF 帧最大长度
#define CROSSFIRE_FRAME_MAXLEN 64

// SBUS 最小值
#define SBUS_LOW 178
// SBUS 最大值
#define SBUS_HIGH 1806
// SBUS 通道数
#define SBUS_CHANNELS_COUNT 16

// UART 定义
#define SERIAL_SBUS Serial1
#define SERIAL_CRSF Serial2

// CRSF 使用 Serial0
#include "crsf.h"
CRSF crsf(SERIAL_CRSF);

uint8_t frame[CROSSFIRE_FRAME_MAXLEN];
uint32_t crossfireChannels[CROSSFIRE_CHANNELS_COUNT];

/*
SBUS数据包的长度为25个字节，包括：
字节[0]：SBUS头，0x0F
字节[1-22]：16个伺服通道，每个伺服通道采用11位编码
字节[23]：
          位7：数字通道17（0x80）
          位6：数字通道18（0x40）
          位5：丢帧（0x20）
          位4：用来激活故障安全（0x10）
          位0-3：n/a
字节[24]：SBUS结束字节，0x00

SBUS的每个RC通道值映射为：
-100％= 173（相当于PWM伺服信号中的1000）
0％= 992（相当于PWM伺服信号中的1500）
100％= 1811（相当于PMW伺服信号中的2000）
*/
// SBUS 使用 Serial1
#include "SBUS.h"
SBUS sbus(SERIAL_SBUS);

// 通道、故障安全和丢帧数据
uint16_t channels[SBUS_CHANNELS_COUNT];
bool failSafe;
bool lostFrame;

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
const unsigned char CRC8TAB[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crc8(const uint8_t *ptr, uint32_t len)
{
  uint8_t crc = 0;
  for (uint32_t i = 0; i < len; i += 1)
  {
    crc = CRC8TAB[crc ^ *ptr++];
  }
  return crc;
}

uint8_t createCrossfireChannelsFrame(uint8_t *frame)
{
  uint8_t *buf = frame;
  *buf++ = MODULE_ADDRESS;
  *buf++ = 24; // 1(ID) + 22 + 1(CRC)
  uint8_t *crc_start = buf;
  *buf++ = CHANNELS_ID;
  uint32_t bits = 0;
  uint8_t bitsavailable = 0;

  for (int i = 0; i < CROSSFIRE_CHANNELS_COUNT; i++)
  {

    // uint32_t val = CROSSFIRE_CENTER;
    // uint32_t val = map(exBus.GetChannel(i),EXBUS_LOW,EXBUS_HIGH,CROSSFIRE_LOW,CROSSFIRE_HIGH);
    uint32_t val = crossfireChannels[i];

    bits |= val << bitsavailable;
    bitsavailable += CROSSFIRE_CH_BITS;
    while (bitsavailable >= 8)
    {
      *buf++ = bits;
      bits >>= 8;
      bitsavailable -= 8;
    }
  }
  *buf++ = crc8(crc_start, 23);

  return buf - frame;
}

// ---------------
static bool toggle = false;

bool IRAM_ATTR runCrossfire(void *timerNo)
{

  memset(frame, 0, sizeof(frame));
  uint8_t length = createCrossfireChannelsFrame(frame);
  SERIAL_CRSF.write(frame, length);
  SERIAL_CRSF.flush();

  // 读取返回数据
  //   telemetryRxBufferCountStream = SERIAL_CRSF.available();
  //   for (int i = 0; i < telemetryRxBufferCountStream; i++) {
  //       processCrossfireTelemetryData(SERIAL_CRSF.read());
  //   }

  // timer interrupt toggles pin LED_BUILTIN
  digitalWrite(LED_BUILTIN, toggle);
  toggle = !toggle;
  delay(1000);
  return true;
}

void ICACHE_RAM_ATTR sbusCallback(volatile uint16_t *data)
{

  for (int8_t i = 0; i < SBUS_CHANNELS_COUNT; i++)
  {
    crossfireChannels[i] = map(data[i], SBUS_LOW, SBUS_HIGH, CROSSFIRE_LOW, CROSSFIRE_HIGH);
  }
  // runCrossfire();
}

// Init ESP32 timer 0
ESP32Timer CrsfTimer(0);

void setup()
{
  // 设置频率
  SERIAL_SBUS.begin(SBUS_BAUD_RATE);
  SERIAL_CRSF.begin(CROSSFIRE_BAUD_RATE, SERIAL_8N1);

  // put your setup code here, to run once:
  // (把你的设置代码放在这里，运行一次：)
  // begin the SBUS communication
  // (开始 SBUS 通信)
  sbus.begin();

  // // If something other than changing the baud of the UART needs to be done, do it here
  // // (如果需要更改 UART 波特率以外的其他操作，请在此处进行)
  // // Serial1.end(); Serial1.begin(500000, SERIAL_8N1, 16, 17);

  // Interval in microsecs
  // 定义时钟激活发送CRSF
  CrsfTimer.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, runCrossfire);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // (把你的主要代码放在这里，重复运行：)
  // look for a good SBUS packet from the receiver
  // (从接收方获得一个好的 SBUS 数据包)
  if (sbus.read(&channels[0], &failSafe, &lostFrame))
  {
    sbusCallback(channels);
  }
}