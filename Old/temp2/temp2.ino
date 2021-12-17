int IRSensor = A0; // connect ir sensor to arduino pin 2



void setup() 
{



  pinMode (IRSensor, INPUT); // sensor pin INPUT
  pinMode (LED_BUILTIN, OUTPUT); // Led pin OUTPUT
  Serial.begin(9600);
}

void loop()
{
  int statusSensor = analogRead (IRSensor);
  Serial.println(statusSensor);
  delay(100);        // delay in between reads for stability
  
  if (statusSensor == 1){
    digitalWrite(LED_BUILTIN, LOW); // LED LOW
  }
  
  else
  {
    digitalWrite(LED_BUILTIN, HIGH); // LED High
  }
  
}
