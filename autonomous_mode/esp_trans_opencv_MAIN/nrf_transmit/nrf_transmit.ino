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
AsyncStream<100> a_serial(&Serial, 'q'); // указали Stream-объект и символ конца

void setup()
{
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
  if (millis() - main_timer > MAIN_PERIOD) {
    if (a_serial.available())
    { // если данные получены
      //Serial.println(a_serial.buf); // выводим их (как char*)
      oled.clear();
      oled.setCursor(0, 0);
      oled.println(a_serial.buf);
      oled.update();
      GParser data(a_serial.buf, ',');
      len_data = data.split();
      // for (byte i = 0; i < len_data; i++) Serial.println(data[i]);
      if (data.equals(0, "#") and data.equals(4, ";") and len_data == 5)
      {
        left_speed = data.getInt(1);
        right_speed = data.getInt(2);
        arm_mode = data.getInt(3);
      }
      else
      {
        //  мб лучше не вмешиватсья????
        left_speed = 0;
        right_speed = 0;
        arm_mode = 0;
      }
    }
    trans_data[0] = left_speed;
    trans_data[1] = right_speed;
    trans_data[2] = arm_mode;
    radio.write(&trans_data, sizeof(trans_data));
  }
}
