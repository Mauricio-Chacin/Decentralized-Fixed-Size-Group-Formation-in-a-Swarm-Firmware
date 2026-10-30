// Definir pines para Motor 1
#include <NewPing.h>
int INPins[4]={4,5,6,7};
#define MAX 200
typedef unsigned char Uchar;
Uchar distFront;
Uchar distWall;
Uchar LastDist;
  NewPing FrontSensor(8,8,200);
  //NewPing WallSensor(9,9,50);
void setup() {
    Serial.begin(9600);
    for(int i = 0; i < 4;i++){
    pinMode(INPins[i], OUTPUT);}
}
void Forward(){
    digitalWrite(INPins[0], HIGH);
    digitalWrite(INPins[1], LOW);
    digitalWrite(INPins[2], HIGH);
    digitalWrite(INPins[3], LOW);
  }
void Stop(){
    digitalWrite(INPins[0], LOW);
    digitalWrite(INPins[1], LOW);
    digitalWrite(INPins[2], LOW);
    digitalWrite(INPins[3], LOW);

  }
void Turn(){
    digitalWrite(INPins[0], HIGH);
    digitalWrite(INPins[1], LOW);
    digitalWrite(INPins[2], LOW);
    digitalWrite(INPins[3], LOW);
    delay(1700);
  }
void Backwards(){
    digitalWrite(INPins[0], LOW);
    digitalWrite(INPins[1], HIGH);
    digitalWrite(INPins[2], LOW);
    digitalWrite(INPins[3], HIGH);
    delay(1000);
  }
void loop() {
  int temporal = 0;
  distFront = 240;
  delay(300);
  temporal = FrontSensor.ping_cm();
  if(temporal != 0){
    distFront = temporal;
    Serial.print("Frontdistance:");
    Serial.println(distFront);
    }
  delay(300);
  if(distFront >= 10 && distFront != 240){
    Backwards();
    Serial.print("Frente");
    }
  else if(distFront <= 10 && distFront != 240){
    Serial.println("Giro");
    Turn();
    }
  else{Stop();}
}
