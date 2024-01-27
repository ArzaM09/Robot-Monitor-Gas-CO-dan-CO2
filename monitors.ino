class MovingAverageco2 {
public:
  MovingAverageco2(int size) {
    size_ = size;
    values_ = new float[size];
    resetco2();
  }

  ~MovingAverageco2() {
    delete[] values_;
  }

  void pushco2(float value) {
    sum_ -= values_[index_];
    values_[index_] = value;
    sum_ += value;
    index_ = (index_ + 1) % size_;
  }

  float getAverageco2() {
    return sum_ / size_;
  }

  void resetco2() {
    for (int i = 0; i < size_; i++) {
      values_[i] = 0.0;
    }
    sum_ = 0.0;
    index_ = 0;
  }

private:
  int size_;
  float* values_;
  float sum_;
  int index_;
};

#define BLYNK_TEMPLATE_ID "TMPL6C4oeur0_"
#define BLYNK_TEMPLATE_NAME "RobotsMonitoring"
#define BLYNK_AUTH_TOKEN "WVLYEpw4hWxqvjEYEIKTUbPwxFcf9X3s"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <DHT.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Hidden Network"; //tinggal ubah ssid(nama) hotspot
char pass[] = "Natural#09"; // tinggal ubah pw hotspot

// Hardware Serial on Mega, Leonardo, Micro...
//#define EspSerial Serial1

// or Software Serial on Uno, Nano...
#include <SoftwareSerial.h>
SoftwareSerial EspSerial(2, 3); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 9600

ESP8266 wifi(&EspSerial);

#define DHTPIN 11         // What digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11     // DHT 11
//#define DHTTYPE DHT22   // DHT 22, AM2302, AM2321
//#define DHTTYPE DHT21   // DHT 21, AM2301

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

int mq = A0;
int R = 10000;
float roco2 = 93671.72;
float a_co2 = 110.7432567;
float b_co2 = -2.856935538;
float ppm_co2 = 418.51;
float MAX_Val = 10;

float minppmco2,maxppmco2;
MovingAverageco2 averageValueco2(MAX_Val);

int ENA =10;
int IN1 =9;
int IN2 =8;
int IN3 =7;
int IN4 =6;
int ENB =5;

bool forward = 0;
bool backward = 0;
bool left = 0;
bool right = 0;
int Speed;
// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
}
void sensorco2()
{

  int adc = analogRead(mq);
  double rs = ((1024.0*R)/adc)-R;
  float rsco2 = rs/roco2;

  if(rsco2 <minppmco2 && rsco2>maxppmco2){
    float ppmco2 = a_co2 * pow((float)rs/(float)roco2, b_co2);
    averageValueco2.pushco2(ppmco2);
    Blynk.virtualWrite(V7, averageValueco2.getAverageco2());
  }
}

BLYNK_WRITE(V0) {
  forward = param.asInt();
}

BLYNK_WRITE(V1) {
  backward = param.asInt();
}

BLYNK_WRITE(V2) {
  left = param.asInt();
}

BLYNK_WRITE(V3) {
  right = param.asInt();
}

BLYNK_WRITE(V4) {
  Speed = param.asInt();
}

void smartcar() {
  if (forward == 1) {
    Forward();
    Serial.println("Forward");
  } else if (backward == 1) {
    Backward();
    Serial.println("Backward");
  } else if (left == 1) {
    Left();
    Serial.println("Left");
  } else if (right == 1) {
    Right();
    Serial.println("Right");
  } else if (forward == 0 && backward == 0 && left == 0 && right == 0) {
    Stop();
    Serial.println("Stop");
  }
}

void setup()
{
  // Debug console
  Serial.begin(9600);

  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);

  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
  // You can also specify server:
  //Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass, IPAddress(192,168,1,100), 8080);

  dht.begin();
  minppmco2 = pow((10/a_co2),1/b_co2);
  maxppmco2 = pow((1000/a_co2),1/b_co2);
  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor);
  timer.setInterval(1000L, sensorco2);

  pinMode(mq, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop()
{
  Blynk.run();
  timer.run();
  smartcar();
}
void Forward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Maju");
}
void Backward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Mundur");
}
void Left() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Kiri");
}
void Right() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Kanan");
}
void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Stop");
}