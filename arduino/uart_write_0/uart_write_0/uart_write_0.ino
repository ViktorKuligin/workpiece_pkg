byte EXPORT  [4];

byte num = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void U_PACK  () {

  EXPORT[0]=0xAA;
  EXPORT[1]=num;
  EXPORT[2]=0x25;
  EXPORT[3]=0xBB;

  num++;
  
  for (int i=0; i<4; i++) 
  { 
    Serial.write(EXPORT[i]);
  }
}

void loop() {
  U_PACK();
  
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(100);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1);                       // wait for a second
}
