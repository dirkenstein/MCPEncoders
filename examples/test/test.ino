#include "MCPEncoders.h"

#define MAX_ENCODERS 14
#define MAX_BUTTONS 14

MCPEncoders * btnenc = new MCPEncoders(MAX_ENCODERS, MAX_BUTTONS);

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  Serial.println("Start");
  Wire.begin();

  const int encmap[] = {7, 8, 9, 4, 5, 10, 12, 0, 3 , 2, 13, 1, 11, 6};
  const int encrmap[] = {1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0};
  const int btnmap [] = {11, 9, 1, 2, 4, 10, 12, 8, 6, 5, 13, 3, 0, 7 };
  const int addrs [] =  {7, 1, 0};
  btnenc->setencmap(encmap);
  btnenc->setencrmap(encrmap);
  btnenc->setbutnmap(btnmap);
  btnenc->begin(4096,  addrs, -1);
}

elapsedMillis ticks;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("start loop");
  while (1){
    btnenc->handle();
  
    for (int x = 0; x< MAX_BUTTONS; x++) {
      if (btnenc->isButtonChanged(x)) {
        Serial.printf("button %d %d %d\n", x, btnenc->getButton(x), btnenc->getButtonTime(x));
      }
    }
    for (int x = 0; x< MAX_ENCODERS; x++) {
  
      if (btnenc->isKnobChanged(x)) {
        Serial.printf("enc %d %d\n", x, btnenc->getKnob(x));
      }
    }  
    if (ticks > 1000) {
      ticks=0;
      Serial.println("tick! ");
      //Serial.print(inttime);
      //Serial.print(" ");
      //Serial.println(digitalRead(ISR_PIN));
      //lastMillis = millis();
      //expander.readINTCAPAB();
      //expander2.readINTCAPAB();
  
    }
    yield();
  }
}
