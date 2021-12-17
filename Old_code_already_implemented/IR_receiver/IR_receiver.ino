#include <IRremote.h>

const int RECV_PIN = A0;
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup(){
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);
}

void loop(){
  if (irrecv.decode()){
        Serial.println(results.value, HEX);
        irrecv.resume();
        //delay(1000);
  }

 
 // IrReceiver.decodedIRData.<fieldname>
  /*
  if (irrecv.decode(&results)){
        Serial.println(results.value, HEX);
        //Serial.println(results.rawbuf);
        irrecv.resume();
        delay(1000);
  }*/
}
