#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define M1_ENCA 34 // oldboard(33), newboard(34) 
// #define M1_ENCA 34 // oldboard(33), newboard(34) 
#define M1_ENCB 35
#define M2_ENCA 32 
#define M2_ENCB 33 // oldboard(34), newboard(33) 

// left side of 6 pin l298 socket
#define M1_EN 26
#define M1_IN1 17 // (TX2)
#define M1_IN2 18
#define M2_EN 25
#define M2_IN1 19 
#define M2_IN2 23

typedef struct  {
    int t1_cmd; // max 255 for 8resolution
    int t2_cmd; // max 255 for 8resolution
} pc_t;
typedef struct  {
    int32_t t1_enc;
    int32_t t2_enc;
    float t1_res;
    float t2_res;
} esp32_t;

pc_t    data_recv;
esp32_t data_send;

IPAddress IP_ap = {192, 168, 1, 1};
IPAddress gateway_ap = {192, 168, 1, 1};
IPAddress NMask_ap = {255, 255, 255, 0};

WiFiUDP udp;
unsigned int localUdpPort = 4210;  // local port to listen on


const int pwm1_channel = 0;
const int pwm2_channel = 1;
const int pwm_freq = 50;
const int pwm_resolution = 8;

volatile int pulse1 = 0;
volatile int pulse2 = 0;

AsyncWebServer server(80);

// Parameter Pengendali
float Kp = 70;
float Kv = 1;

float tetha1_act = 0;
float tetha1_old_act = 0;
volatile float tetha1_err = 0;
volatile float tetha1_err_old = 0;
volatile float dutyCycle1 = 0;

float tetha2_act = 0;
float tetha2_old_act = 0;
volatile float tetha2_err = 0;
volatile float tetha2_err_old = 0;
volatile float dutyCycle2 = 0;

float V_1 = 0.1;
float V_2 = 0.1;
float Vmax = 1000;
float Vmin = -1000;

short frequency = 50;
short periode = 3;
int dt = 0.017;


void setupWeb(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
     request->send(200, "text/plain", "Sedang dipakai raihan n");
  });
  server.onNotFound([](AsyncWebServerRequest *request){
     request->send(404, "text/plain", "Not found");
  });  

  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  server.begin();
}

void IRAM_ATTR encoder1() {
  if(digitalRead(M1_ENCA) && digitalRead(M1_ENCB)) {
    pulse1++;
  }

  else if(digitalRead(M1_ENCA) && !digitalRead(M1_ENCB)) {
    pulse1--;
  }

  else if(!digitalRead(M1_ENCA) && digitalRead(M1_ENCB)) {
    pulse1--;
  }

  else if(!digitalRead(M1_ENCA) && !digitalRead(M1_ENCB)) {
    pulse1++;
  }
}


void IRAM_ATTR encoder2() {
  if(digitalRead(M2_ENCA) && digitalRead(M2_ENCB)) {
    pulse2++;
  }

  else if(digitalRead(M2_ENCA) && !digitalRead(M2_ENCB)) {
    pulse2--;
  }

  else if(!digitalRead(M2_ENCA) && digitalRead(M2_ENCB)) {
    pulse2--;
  }

  else if(!digitalRead(M2_ENCA) && !digitalRead(M2_ENCB)) {
    pulse2++;
  }
}

//void IRAM_ATTR encoder2() {
// // two times reading
// pulse2 += digitalRead(M2_ENCA) ? 1 : -1;
//}


void setup() {
  Serial.begin(115200);
  // initialize wifi
  uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
  uint16_t chip = (uint16_t)(chipid >> 32);
  String s_ssid = "ROBOT-" + String(chip,HEX);
  String s_password = "12345678";
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(s_ssid.c_str(), s_password.c_str(),3,0,20); //ssid, pw, ch, hidden, maxconn
  WiFi.softAPConfig(IP_ap, IP_ap, NMask_ap);
  
  // setup OTA
  setupWeb();

  // Starting UDP
  udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // setup encoder 
  pinMode(M1_ENCA, INPUT_PULLUP);
  pinMode(M1_ENCB, INPUT_PULLUP);
  pinMode(M2_ENCA, INPUT_PULLUP);
  pinMode(M2_ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENCB), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENCB), encoder2, CHANGE);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M1_EN, OUTPUT);
  pinMode(M2_EN, OUTPUT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // receive incoming UDP packets
    Serial.printf("Recv %d bytes from %s, port %d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
    int len = udp.read((uint8_t *)&data_recv, sizeof(data_recv));
    // Serial.printf("t1cmd: %.4f in %d ms OR DC %d %d\n", data_recv.t1_final, data_recv.t1_msec, data_recv.t1_cmd, data_recv.t2_cmd);


    tetha1_err = data_recv.t1_cmd;
    tetha2_err = data_recv.t2_cmd;

    V_1 = Kp * tetha1_err + Kv / dt * (tetha1_err - tetha1_err_old);
    V_2 = Kp * tetha2_err + Kv / dt * (tetha2_err - tetha2_err_old);

    if (V_1 > Vmax) {
      V_1 = Vmax;
    }

    else if (V_1 < Vmin) {
      V_1 = Vmin;
    }

    if (V_2 > Vmax) {
      V_2 = Vmax;
    }

    else if (V_2 < Vmin) {
      V_2 = Vmin;
    }

    tetha1_err_old = tetha1_err;
    tetha2_err_old = tetha2_err;

    dutyCycle1 = map(V_1, -1000, 1000, 150, 255);
    dutyCycle2 = map(V_2, -1000, 1000, 150, 255);

    
    if(dutyCycle1 > 255){
      dutyCycle1 = 255;
    }

    if(dutyCycle2 > 255){
      dutyCycle2 = 255;
    }
  
    if(V_1 > 0){
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
      analogWrite(M1_EN,dutyCycle1);
    } 
    
    else if (V_1 < 0){
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
      analogWrite(M1_EN,dutyCycle1);
    } 
    
    else {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, LOW);
      analogWrite(M1_EN,0);
    }

    if(V_2 > 0){
      digitalWrite(M2_IN1, HIGH);
      digitalWrite(M2_IN2, LOW);
      analogWrite(M2_EN,dutyCycle2);
    } 
    
    else if (V_2 < 0){
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, HIGH);
      analogWrite(M2_EN,dutyCycle2);
    } 
    
    else {
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, LOW);
      analogWrite(M2_EN,0);
    }
    
    // send back a reply, to the IP address and port we got the packet from
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    // dummy data should be assigned with real value
    data_send.t1_res= (pulse1 / 823) * 360;
    data_send.t2_res= (pulse2 / 823) * 360;
    data_send.t1_enc= pulse1;
    data_send.t2_enc= pulse2;
    udp.write((uint8_t *)&data_send, sizeof(data_send));
    udp.endPacket();
      // below just for debugging, for realtime response better use short delay
  }
  Serial.printf("pulsa1 : %d, pulsa2 : %d\n",(int)pulse1, (int)pulse2);
  delay(100);
}
