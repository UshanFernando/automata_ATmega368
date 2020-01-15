#include <Arduino.h>
#include <SimpleDHT.h>
SimpleDHT11 DHT;

#define DHT11_PIN 7
byte temperature = 0;
byte humidity = 0;

#define MOTION_PIN 4
bool en_motion = 0;

String incoming = "";   // for incoming serial string data
int L1 = 12;
int L2 = 11;
int L3 = 10;
int L4 = 9;

void setup() {
    Serial.begin(115200); 
//    SUART.begin(115200); //enable SUART Port

    pinMode(L1,OUTPUT);  
    pinMode(L2,OUTPUT);  
    pinMode(L3,OUTPUT);  
    pinMode(L4,OUTPUT); 

    digitalWrite(L1,HIGH);
    digitalWrite(L2,HIGH);
    digitalWrite(L3,HIGH);
    digitalWrite(L4,HIGH);
}

void sendSensorData(){

  DHT.read(DHT11_PIN, &temperature, &humidity, NULL);

  Serial.print('>');  
  Serial.print(temperature,DEC);
  Serial.print(',');                  
  Serial.println(humidity, DEC);
    
}

void detectGesture(){

  int stat = 1;
  stat = digitalRead(MOTION_PIN);

  if (stat == 0 && en_motion == 0)
  {
    en_motion = 1;
    Serial.println("L1_SW");
    // delay(1000);
  }
  else if (stat == 1 && en_motion == 1)
  {
    en_motion = 0;
  } 
  
}

void loop() {

  sendSensorData();
  detectGesture();

  if(Serial.available() > 0) {
      // read the incoming:
      incoming = Serial.readString();
 
      //DEBUGGING
      // Serial.println(incoming); 
      incoming.trim();  
      
      if (incoming == "L1_ON") {

        digitalWrite(L1,LOW);
        Serial.println("L1_ON_OK");

      } else if (incoming=="L1_OFF") {

        digitalWrite(L1,HIGH);
        Serial.println("L1_OFF_OK");

      } else if (incoming=="L2_ON") {

        digitalWrite(L2,LOW);
        Serial.println("L2_ON_OK");

      } else if (incoming=="L2_OFF") {

        digitalWrite(L2,HIGH);
        Serial.println("L2_OFF_OK");

      } else if (incoming=="L3_ON") {

        digitalWrite(L3,LOW);
        Serial.println("L3_ON_OK");

      } else if (incoming=="L3_OFF") {

        digitalWrite(L3,HIGH);
        Serial.println("L3_OFF_OK");

      } else if (incoming=="L4_ON") {

        digitalWrite(L4,LOW);
        Serial.println("L4_ON_OK");

      } else if (incoming=="L4_OFF") {

        digitalWrite(L4,HIGH);
        Serial.println("L4_OFF_OK");

      }  
      else {
        //junk
        // Serial.println("NO Command");
        incoming = "";
      }
      Serial.flush();
    }
  delay(1000);
}