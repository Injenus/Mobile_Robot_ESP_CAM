/*
   This is the sketch for Inj_Bot for autonomus mode by NRF
   nrf data =left, rignt, arm_mode
    руку фиксируем, площадку вертим
    вертим всегда, если не запрещено

*/

// Скорость колеса от -100 до 100 в абстрактных единцах
#include <I2Cdev.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
#include <ServoDriverSmooth.h>

const byte WHEEL_NUM = 2;
int wheel_pin[WHEEL_NUM] = {9, 10};

struct Servo_val
{
  int const dead_zone = 21; // +- relative to 90
  int const min_v = 0;      // >=0 <90    55 is good
  int const stop_v = 90;    //
  int const max_v = 180;    // >90 <=180   125 is good
};
Servo_val sv;
Servo servo[WHEEL_NUM];
int pin_arm[4] = {0, 2, 4, 6};
ServoDriverSmooth arm_servo[4](0x40);
int arm_pos[4] = {0, 90, 90, 90}; // 0 -var , 1-3 - const
int arm_mode = 0;

struct Arm_pos {
  int off = 35;
  int pos1 = 115;
  int pos2 = 140;
  bool flag = true;
};
Arm_pos apos;

byte state_led = 18;

struct Mcs
{
  int const min_prd = 600;
  int const stop_prd = 1500;
  int const max_prd = 2400;
};
Mcs mcs;

struct RGB
{
  int r = 2;
  int g = 4;
  int b = 3;
};
RGB rgb;

RF24 radio(7, 8);                                                           // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб
const byte LEN_DATA = 3;
int data[LEN_DATA]; // l r arm_mode
byte pipeNo;

bool is_on_tr = false;

#define MAIN_PERIOD 3
uint32_t main_timer = 0;
uint32_t nrf_t = 0;
uint32_t NRF_P = 350;
bool main_flag = false;
int m_c = 0;

uint32_t arm_timer = 0;
uint32_t ARM_P = 333;

int l = 0;
int r = 0;

void setup()
{
  //  for (int i = 2; i < 22; i++)
  //  {
  //    pinMode(i, OUTPUT);
  //  }
  Serial.begin(9600);       // открываем порт для связи с ПК
  radio.begin();            // активировать модуль
  radio.setAutoAck(1);      // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);  // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload(); // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32); // размер пакета, в байтах

  radio.openReadingPipe(1, address[0]); // хотим слушать трубу 0
  radio.setChannel(0x6a);               // выбираем канал (в котором нет шумов!)

  radio.setPALevel(RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  radio.powerUp();                 // начать работу
  radio.startListening();          // начинаем слушать эфир, мы приёмный модуль

  for (int i = 0; i < WHEEL_NUM; i++)
  {
    servo[i].attach(wheel_pin[i], mcs.min_prd, mcs.max_prd);
    servo[i].write(90);
  }
  pinMode(rgb.r, OUTPUT);
  pinMode(rgb.g, OUTPUT);
  pinMode(rgb.b, OUTPUT);

  for (int i = 0; i < 4; i++)
  {
    arm_servo[i].attach(pin_arm[i]);
    arm_servo[i].setAutoDetach(false);
    arm_servo[i].setSpeed(350);
    arm_servo[i].setAccel(1.0);
  }
  //arm_servo[0].setAccel(0.95);
  arm_servo[0].write(apos.off);
  arm_servo[1].write(90);
  arm_servo[2].write(90);
  arm_servo[3].write(90);

  digitalWrite(rgb.g, 1);
  delay(100);
  digitalWrite(rgb.g, 0);
}

void loop()
{
    arm_servo[0].tick();
  if (!is_on_tr)
  { // если не примаем, ставим иниты кроме руки
    for (byte i = 0; i < LEN_DATA - 1; i++)
    {
      data[i] = 0;
    }
  }
  if (millis() - nrf_t > NRF_P) {
    is_on_tr = false;
    digitalWrite(rgb.r, 1);
  }
  if (radio.available(&pipeNo))
  { // слушаем эфир со всех труб
    nrf_t = millis();
    is_on_tr = true;
    digitalWrite(rgb.r, 0);

    radio.read(&data, sizeof(data)); // чиатем входящий сигнал
    //    Serial.print("Recieved: ");
    //    Serial.print(data[0]);
    //    for (byte i = 1; i < LEN_DATA; i++)
    //    {
    //      Serial.print(',');
    //      Serial.print(data[i]);
    //    }
    //    Serial.println(';');

  }


  arm_mode = data[2];

  if (data[0] < 0)
  {
    l = map(data[0], -100, 0, sv.min_v, sv.stop_v - sv.dead_zone);
  }
  else if (data[0] > 0)
  {
    l = map(data[0], 1, 100, sv.stop_v + sv.dead_zone, sv.max_v);
  }
  else
  {
    l = 90;
  }

  if (data[1] < 0)
  {
    r = map(data[1], -100, 0, sv.min_v, sv.stop_v - sv.dead_zone);
  }
  else if (data[1] > 0)
  {
    r = map(data[1], 1, 100, sv.stop_v + sv.dead_zone, sv.max_v);
  }
  else
  {
    r = 90;
  }

  servo[0].write(l);
  servo[1].write(r);
  set_arm_mode();

  //  Serial.print(l);
  //  Serial.print(',');
  //  Serial.print(r);
  //  Serial.print(',');
  //  Serial.println(arm_mode);

  if (millis() - arm_timer > ARM_P) {
    arm_timer = millis();
    apos.flag = !apos.flag;
  }

}


void set_arm_mode()
{
  digitalWrite(rgb.b, arm_mode);
  if (arm_mode == 0) {
    arm_servo[0].setTargetDeg(apos.off);
  }
  else if (arm_mode == 1) {
    if (apos.flag) {
      //      arm_servo[0].setTargetDeg(apos.pos1);
      arm_servo[0].write(apos.pos1);
    }
    else {
      //      arm_servo[0].setTargetDeg(apos.pos2);
      arm_servo[0].write(apos.pos2);
    }
  }
}
