#include <WiFi.h>

const char *ssid = "Inj_32"; //"Inj_Phone";//"MTS_GPON_930a70"; //"DESKTOP-238VSIG 4784";  "Inj_32";
const char *password = "esp32injenus"; //"12255555"; //"Bkbi6j7g"; //"o6267#7F";   "esp32injenus";
const int serverPort = 2903;

WiFiServer server(serverPort);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
//    Serial.println("Client connected");

    while (client.connected()) {
      if (client.available()) {
        String data = client.readStringUntil('\n');
        Serial.print(data+"\n");
        // Здесь вы можете обрабатывать полученные данные

//        // Отправляем ответ клиенту
//        client.print("Data received: ");
//        client.println(data);
      }
    }

//    client.stop();
//    Serial.println("Client disconnected");
  }
}
