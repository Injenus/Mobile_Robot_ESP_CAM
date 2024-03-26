/*
   This is the sketch for Inj_Bot for remote control mode
*/

// Скорость колеса от -100 до 100 в абстрактных единцах

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>

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

struct Mcs {
  int const min_prd = 600;
  int const stop_prd = 1500;
  int const max_prd = 2400;
};
Mcs mcs;

RF24 radio(7, 8);                                                           // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб
const byte LEN_DATA = 2;
int data[LEN_DATA]; // xl r
byte pipeNo;

bool is_on_tr = false;

#define MAIN_PERIOD 5
uint32_t main_timer = 0;

void setup()
{
  for (int i = 2; i < 22; i++)
  {
    pinMode(i, OUTPUT);
  }
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
}

void loop()
{
  if (radio.available(&pipeNo))
  { // слушаем эфир со всех труб
    if (!is_on_tr)
    {
      is_on_tr = true;
    }
    radio.read(&data, sizeof(data)); // чиатем входящий сигнал
  }
  if (!is_on_tr)
  { // если не примаем, ставим иниты
    for (byte i = 0; i < LEN_DATA; i++)
    {
      data[i] = 0;
    }
  }

  Serial.print("Recieved: ");
  Serial.print(data[0]);
  for (byte i = 1; i < LEN_DATA; i++)
  {
    Serial.print(',');
    Serial.print(data[i]);
  }
  Serial.println(';');

  if (millis() - main_timer > MAIN_PERIOD)
  {
    main_timer = millis();

    int l = 0;
    int r = 0;

    if (data[0] < 0)
    {
      l = map(data[0], -100, 0, sv.min_v, sv.stop_v - sv.dead_zone);
    }
    else if (data[0] > 0)
    {
      l = map(data[0], 1, 100, sv.stop_v + sv.dead_zone, sv.max_v);
    }
    else {
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
    else {
      r = 90;
    }

    servo[0].write(l);
    servo[1].write(r);

    Serial.print(l);
    Serial.print(',');
    Serial.println(r);
  }
}
