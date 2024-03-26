#include<WiFi.h>;

void setup(){
  Serial.begin(115200);
  WiFi.softAP("Inj_32", "esp32injenus");
  Serial.println(WiFi.softAPIP());
}

void loop(){
  Serial.print("Num of conn: ");
  Serial.println(WiFi.softAPgetStationNum());
}
