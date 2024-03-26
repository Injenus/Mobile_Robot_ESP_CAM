/*
   This is sketch for Remote for remote control mode
*/

#include <SPI.h>      // библиотека для работы с шиной SPI
#include "nRF24L01.h" // библиотека радиомодуля
#include "RF24.h"     // ещё библиотека радиомодуля

#define MAIN_PERIOD 10
uint32_t main_timer = 0;

RF24 radio(9, 10);
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб
byte counter;
#define IN_DATA 9
#define OUT_DATA 2
byte data_pin[IN_DATA] = {14, 15, 17, 16, 2, 4, 5, 7, 6}; // x1 y1 x2 y2 btn1 btn2, mode1(+l-r), mode2(-l+r), mode3(+l+r)
int mode = 1;
int joy_data[IN_DATA];
int trans_data[OUT_DATA] = {0, 0}; // l, r
#define LED 3

void setup()
{
  for (int i = 2; i < 20; i++)
  {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600); // открываем порт для связи с ПК

  radio.begin();            // активировать модуль
  radio.setAutoAck(1);      // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);  // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload(); // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32); // размер пакета, в байтах

  radio.openWritingPipe(address[0]); // мы - труба 0, открываем канал для передачи данных
  radio.setChannel(0x6a);            // выбираем канал (в котором нет шумов!)

  radio.setPALevel(RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  // должна быть одинакова на приёмнике и передатчике! при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();       // начать работу
  radio.stopListening(); // не слушаем радиоэфир, мы передатчик

  for (byte i = 0; i < IN_DATA; i++)
  {
    if (i < 4)
    {
      pinMode(data_pin[i], INPUT);
    }
    else
    {
      pinMode(data_pin[i], INPUT_PULLUP);
    }
  }
  digitalWrite(LED, HIGH);
  Serial.println("Init");
}

void loop()
{
  if (millis() - main_timer > MAIN_PERIOD)
  {
    main_timer = millis();
    digitalWrite(LED, !digitalRead(LED));
    for (byte i = 0; i < IN_DATA; i++)
    {
      if (i < 4)
      {
        joy_data[i] = analogRead(data_pin[i]);
      }
      else
      {
        joy_data[i] = digitalRead(data_pin[i]);
      }
    }

    simple_set();

    radio.write(&trans_data, sizeof(trans_data));

    Serial.print(trans_data[0]);
    Serial.print(',');
    Serial.println(trans_data[1]);
  }
}

void simple_set()
{
  // преобразуем стики от 0..1023 к -100, 100
  int LX = map(joy_data[0], 1023, 0, -25, 25);
  int LY = map(joy_data[1], 1023, 0, -25, 25);

  LX = map(LX, -25, 25, -100, 100);
  LY = map(LY, -25, 25, -100, 100);

  // танковая схема
  if (joy_data[5] == 0)
  {
    LX = LX / 20;
    LY = LY / 20;
  }
  int dutyR = LY + LX;
  int dutyL = LY - LX;
  set_mode();

  if (joy_data[5] == 0)
  {
    int k = 2;
    dutyR = constrain(dutyR, -10/k, 10/k);
    dutyL = constrain(dutyL, -10/k, 10/k);

    switch (mode)
    {
      case 1:
        dutyR = map(dutyR, -10/k, 10/k, -10/k, 10/k);
        dutyL = map(dutyL, -10/k, 10/k, 10/k, -10/k);
        break;
      case 2:
        dutyR = map(dutyR, -10/k, 10/k, 10/k, -10/k);
        dutyL = map(dutyL, -10/k, 10/k, -10/k, 10/k);
        break;
      case 3:
        dutyR = map(dutyR, -10/k, 10/k, -10/k, 10/k);
        dutyL = map(dutyL, -10/k, 10/k, -10/k, 10/k);
        break;
      case 4:
        dutyR = map(dutyR, -10/k, 10/k, -10/k, 10/k);
        dutyL = map(dutyL, -10/k, 10/k, -10/k, 10/k);
        break;
    }
  }
  else
  {
    dutyR += 4;
    dutyL += 4;
    dutyR = constrain(dutyR, -100, 100);
    dutyL = constrain(dutyL, -100, 100);

    switch (mode)
    {
      case 1:
        dutyR = map(dutyR, -100, 100, -100, 100);
        dutyL = map(dutyL, -100, 100, 100, -100);
        break;
      case 2:
        dutyR = map(dutyR, -100, 100, 100, -100);
        dutyL = map(dutyL, -100, 100, -100, 100);
        break;
      case 3:
        dutyR = map(dutyR, -100, 100, -100, 100);
        dutyL = map(dutyL, -100, 100, -100, 100);
        break;
      case 4:
        dutyR = map(dutyR, -100, 100, -100, 100);
        dutyL = map(dutyL, -100, 100, -100, 100);
        break;
    }
  }

  trans_data[0] = dutyL;
  trans_data[1] = dutyR;
}

void set_mode()
{
  if (joy_data[6] == 0)
  {
    mode = 1;
  }
  else if (joy_data[7] == 0)
  {
    mode = 2;
  }
  else if (joy_data[8] == 0)
  {
    mode = 3;
  }
}
