#include <Wire.h>
#include <tca8418.h>

KEYS Keypad;

volatile bool KeyInt=false;

void KeyISR(void) {  //Keypad Interrupt Service Routine
  KeyInt = true;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Configure for a 4X4 matrix on COL0-COL4, ROW0-ROW4 and enable interrupts
  Keypad.begin(ROW0|ROW1|ROW2|ROW3, COL0|COL1|COL2|COL3, 
                 CFG_KE_IEN|CFG_OVR_FLOW_IEN|CFG_INT_CFG|CFG_OVR_FLOW_M); 
                 
                 
  Keypad.enableInterrupt(2, KeyISR);  //Arg1= Arduino Pin number INT is connected to. Arg2= Interrupt Routine

  Keypad.pinMode(15, OUTPUT);
  Keypad.pinMode(16, OUTPUT);
  Keypad.pinMode(17, OUTPUT);
  
  Keypad.digitalWrite(15, true);
  Keypad.digitalWrite(16, false);
  Keypad.digitalWrite(17, false);
  
}

void loop() {
  //Check for Interrupt flag and process
  if(KeyInt) {
    uint8_t key;

    key=Keypad.readKeypad(); //Get first keycode from FIFO
    Serial.print("Keyboard ISR...Key:");
    Serial.print((key&0x7F), HEX); //print keycode masking the key down/key up bit (bit7)
    if(key & 0x80) {
      Serial.println(" key down");
    } else {
      Serial.println(" key up");
    } 
    KeyInt=false; //Reset Our Interrupt flag
    Keypad.clearInterruptStatus(); //Reset TCA8418 Interrupt Status Register.
  }
  //Do other processing
  ;
}
