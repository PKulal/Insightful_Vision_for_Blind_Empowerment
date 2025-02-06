#include "BluetoothSerial.h"
#include "DHT.h"
#define DHTPIN 27
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//--------------------------------------------------------------------------------
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial bluetooth;

#define LED_STATE_BLUE 2

#define Button_PIN_1 18
#define Button_PIN_2 16
#define Button_PIN_3 17

#define US_Echo_1 22
#define US_Trig_1 23

#define US_Echo_2 32
#define US_Trig_2 33

#define US_Echo_3 26
#define US_Trig_3 25


//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration;
float distanceCm;
float distanceInch;


void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_OPEN_EVT) {          // BT: Checks if the SPP connection is open, the event comes// event == Client connected
    Serial.println("Client Connected");     // BT: Write to the serial monitor
  } else if (event == ESP_SPP_CLOSE_EVT) {  // BT: event == Client disconnected
    Serial.println("Client Disconnected");  // BT: Write to the serial monitor
  }
}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

struct Button_1 {
  const uint8_t PIN;
  bool pressed;
};
Button_1 button1 = { Button_PIN_1, false };

struct Button_2 {
  const uint8_t PIN;
  bool pressed;
};
Button_2 button2 = { Button_PIN_2, false };

struct Button_3 {
  const uint8_t PIN;
  bool pressed;
};
Button_3 button3 = { Button_PIN_3, false };

//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;
unsigned long last_button_time = 0;

void IRAM_ATTR Button_1_isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    button1.pressed = true;
    last_button_time = button_time;
  }
}

void IRAM_ATTR Button_2_isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    button2.pressed = true;
    last_button_time = button_time;
  }
}

void IRAM_ATTR Button_3_isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    button3.pressed = true;
    last_button_time = button_time;
  }
}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  bluetooth.register_callback(Bt_Status);
  if (!bluetooth.begin("ESP32_Blind")) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized... Bluetooth Device is Ready to Pair...");
  }
  Serial.println("RS_ESP32 Begin");

  pinMode(LED_STATE_BLUE, OUTPUT);
  pinMode(button1.PIN, INPUT_PULLUP);
  pinMode(button2.PIN, INPUT_PULLUP);
  pinMode(button3.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, Button_1_isr, FALLING);
  attachInterrupt(button2.PIN, Button_2_isr, FALLING);
  attachInterrupt(button3.PIN, Button_3_isr, FALLING);

  pinMode(US_Trig_1, OUTPUT);  // Sets the trigPin as an Output
  pinMode(US_Echo_1, INPUT);   // Sets the echoPin as an Input

  pinMode(US_Trig_2, OUTPUT);  // Sets the trigPin as an Output
  pinMode(US_Echo_2, INPUT);   // Sets the echoPin as an Input

  pinMode(US_Trig_3, OUTPUT);  // Sets the trigPin as an Output
  pinMode(US_Echo_3, INPUT);   // Sets the echoPin as an Input
  dht.begin();

}

void loop() {
  if (bluetooth.hasClient() == true) {
    // Serial.println("bluetooth Connected");

    int t_interval = 200;
    digitalWrite(LED_STATE_BLUE, HIGH);

    if (button1.pressed) {
      Serial.println("Button 1");
      bluetooth.println("1");
      button1.pressed = false;
    }

    if (button2.pressed) {
      Serial.println("Button 2");
      bluetooth.println("2");
      button2.pressed = false;
    }

    if (button3.pressed) {
      Serial.println("Button 3");
      bluetooth.println("3");
      button3.pressed = false;
    }

    float Dis_1 = Distance_Calculation(US_Trig_1, US_Echo_1);
    Serial.print("Distance (cm) 1: ");
    Serial.println(Dis_1);
    if (Dis_1 <= 20) {
      Serial.println("LEFT");
      bluetooth.println("LEFT");
    }

    delay(500);           // 1000 = 1second(1000 milli second)

    float Dis_2 = Distance_Calculation(US_Trig_2, US_Echo_2);
    Serial.print("Distance (cm) 2: ");
    Serial.println(Dis_2);
    if (Dis_2 <= 20) {
      Serial.println("CENTER");
      bluetooth.println("CENTER");
    }
    delay(500);

    float Dis_3 = Distance_Calculation(US_Trig_3, US_Echo_3);
    Serial.print("Distance (cm) 3: ");
    Serial.println(Dis_3);
    if (Dis_3 <= 20) {
      Serial.println("RIGHT");
      bluetooth.println("RIGHT");
    }
    delay(500);
    float t = dht.readTemperature();
    Serial.print(t);
    Serial.println("ºC");
    bluetooth.print("Temp : ");
    bluetooth.println(t);

    delay(1000);
  } else {
    if (button1.pressed) {
      Serial.println("Button 1");
      button1.pressed = false;
    }

    if (button2.pressed) {
      Serial.println("Button 2");
      button2.pressed = false;
    }

    if (button3.pressed) {
      Serial.println("Button 3");
      button3.pressed = false;
    }
  }
}

float Distance_Calculation(int US_Trig, int US_Echo) {
  // Clears the trigPin
  digitalWrite(US_Trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(US_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_Trig, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(US_Echo, HIGH);

  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;

  return distanceCm;
}
