/*
   This is sketch for Transmitter for autu mode Inj bot (with ESP as Wifi hot spot n esp cam)
   message:
   #,left,right,arm_mode,;   0,  1  2  3    4

   left- right_speed = -100 .. 100
   arm_mode  0 - init(passive)  1 - active Func
   #,0,0,0,;
*/

#include <SPI.h>      // библиотека для работы с шиной SPI
#include "nRF24L01.h" // библиотека радиомодуля
#include "RF24.h"     // ещё библиотека радиомодуля
#include <AsyncStream.h>
#include <GParser.h>
#include <GyverOLED.h>
GyverOLED<SSD1306_128x32> oled;

#define MAIN_PERIOD 5
uint32_t main_timer = 0;

RF24 radio(9, 10);
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб
byte counter;
#define OUT_DATA 3
int trans_data[OUT_DATA] = {0, 0, 0}; // l, r, arm_mode
#define LED 3
int len_data;
int left_speed = 0;
int right_speed = 0;
int arm_mode = 0;

char strData[100];
boolean recievedFlag;
int cd = 0;
int c1 = 0;
int c2 = 0;
int c3 = 0;

char temp1[2] = {'0', '\0'};
char temp2[3] = {'1', '0', '\0'};


void setup()
{
  Serial.begin(115200); // открываем порт для связи с ПК

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
  pinMode(LED, OUTPUT);
  oled.init();  // инициализация
  //oled.flipV(1);
  //oled.flipH(1);
  oled.textMode(BUF_ADD);
  oled.setScale(4);
  // --------------------------
  // настройка скорости I2C
  //Wire.setClock(800000L);   // макс. 800'000
  // --------------------------
  oled.clear();   // очистить дисплей (или буфер)
  oled.setCursorXY(16, 0);
  oled.print("INIT");
  oled.update();
  oled.setScale(1);
  oled.setCursor(0, 0);
  digitalWrite(LED, 1);
  delay(50);
  digitalWrite(LED, 0);
}

void loop()
{
  if (!recievedFlag) {
    if (Serial.available() > 0) {
      char t = (char)Serial.read();
      Serial.print(t);
      if (t == 'q') {
        recievedFlag = true;
        cd = 0;
//        delayMicroseconds(2);
        //Serial.flush();
      }
      else {
        strData[cd] = t;
        cd++;
//        delayMicroseconds(2);
      }

    }
  }
  if (recievedFlag) {                      // если данные получены
    oled.clear();
    oled.setCursor(0, 0);
    oled.print(strData);

    if (strData[0] == '#' and strData[1] == ',' and strData[4] == ',' and strData[7] == ',' and strData[9] == ',' and strData[10] == ';') {
      temp2[0] = strData[2]; temp2[1] = strData[3];
      left_speed = atoi(temp2) - 10;
      temp2[0] = strData[5]; temp2[1] = strData[6];
      right_speed = atoi(temp2) - 10;
      temp1[0] = strData[8];
      arm_mode = atoi(temp1);
      digitalWrite(LED, 0);
    }
    else
    {
      left_speed = 0;
      right_speed = 0;
      digitalWrite(LED, 1);
    }
    trans_data[0] = left_speed;
    trans_data[1] = -right_speed;
    trans_data[2] = arm_mode;
    radio.write(&trans_data, sizeof(trans_data));
    oled.setCursor(0, 1);
    oled.print(left_speed);
    oled.setCursor(0, 2);
    oled.print(right_speed);
    oled.setCursor(0, 3);
    oled.print(arm_mode);
    oled.update();

    recievedFlag = false;                  // опустить флаг
  }
}
