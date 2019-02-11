#include <SoftwareSerial.h>

#define DEBOUNCE 30
#define NUMBUTTONS sizeof(buttons)
#define NUMLEDS sizeof(leds)
SoftwareSerial HC12(10, 11);         // HC-12 TX Pin, HC-12 RX Pin

byte incomingByte;
String readBuffer = "";
byte temp = 100;
byte buttons[] = {2,3,4,8};
byte leds[] = {5,6,7};
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
byte previous_keystate[NUMBUTTONS], current_keystate[NUMBUTTONS];

void setup() {
  byte i,j;
  HC12.begin(9600);                     // Open serial port to HC12
  //Serial.begin(9600);
  for (i=0; i< NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }
  for (j=0; j< NUMLEDS; j++) {
    pinMode(leds[j], OUTPUT);
    digitalWrite(leds[j], HIGH);
  }
}



void loop() {
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    readBuffer += char(incomingByte);    // Add each byte to ReadBuffer string variable
  }
  byte thisSwitch=thisSwitch_justPressed();
  switch(thisSwitch)
  {  
  case 0:  
    HC12.println("S0301E");
    digitalWrite(leds[0], LOW);
    digitalWrite(leds[1], HIGH);
    digitalWrite(leds[2], HIGH);
    //temp = 0;
    delay(50);
    break;
  case 1: 
    HC12.println("S0302E");
    digitalWrite(leds[1], LOW);
    digitalWrite(leds[0], HIGH);
    digitalWrite(leds[2], HIGH);
    //temp = 1;
    delay(50);
    break;
  case 2: 
    HC12.println("S0303E");
    digitalWrite(leds[0], HIGH);
    digitalWrite(leds[1], HIGH);
    digitalWrite(leds[2], LOW);
    //temp = 2;
    delay(50);
    break;
  }
//  if (temp == 0){
//    if (readBuffer == "ok"){
//      temp = 100;
//      Serial.println("Done");
//      readBuffer = "";
//    }
//    else{
//      HC12.println("button 1 pressed");
//      delay(500);
//    }
//  }
}


//##########################################################

void check_switches()
{
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;
  if (millis() < lasttime) {
    // we wrapped around, lets just try again
    lasttime = millis();
  }
  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return; 
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();
  for (index = 0; index < NUMBUTTONS; index++) {
    justpressed[index] = 0;       //when we start, we clear out the "just" indicators
    justreleased[index] = 0;
    currentstate[index] = digitalRead(buttons[index]);   //read the button
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
        // just pressed
        justpressed[index] = 1;
        //Serial.println(justpressed[index]);
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
        justreleased[index] = 1; // just released
      }
      pressed[index] = !currentstate[index];  //remember, digital HIGH means NOT pressed
    }
    previousstate[index] = currentstate[index]; //keep a running tally of the buttons
  }
}
 
byte thisSwitch_justPressed() {
  byte thisSwitch = 255;
  check_switches();  //check the switches &amp; get the current state
  for (byte i = 0; i < NUMBUTTONS; i++) {
    current_keystate[i]=justpressed[i];
    if (current_keystate[i] != previous_keystate[i]) {
      if (current_keystate[i]) thisSwitch=i;
    }
    previous_keystate[i]=current_keystate[i];
  }  
  return thisSwitch;
}


//##########################################################

