/***********************************************************************
 * 
 * ==============================
 * =  Rotary Dimmer, 8-bit PWM  =
 * ==============================
 * 
 * Copyright 2017 Michael Kwun
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 ***********************************************************************/

/*
 * Dimmer switch for 12V LED strip lights, using a cheap rotary encoder,
 * and a CIE 1931 dimming curve.
 * 
 * NOTE THAT THIS SKETCH IS WRITTEN FOR AN ARDUINO NANO.  In loop(), I  
 * directly read inputs using the PIND register.  These need to be 
 * carefully reviewed and adjusted as necessary if a different board is 
 * used, or if different pins are used. 
 *
 * I have a 12V power supply for my lights.  This is also used to power 
 * Nano.  My strip lights are a single color, so I need one output pin 
 * to control them, which I do using an N-channel MOSFET.
 *
 * Connections:
 *  > 12V+ is connected to VIN, and to V+ on the strip lights.
 *  > GND on the power supply is connected to ground on the Nano.
 *  > My rotary encoder is connected to D2 & D3 on the Nano. 
 *  > The pushbutton on the encoder is connected to D4 on the Nano.
 *  > D5 on the Nano is connected to gate on an N-channel MOSFET.
 *  > Drain on the MOSFET is connected to V- on the strip lights.
 *  > Source on the MOSFET is connected to ground on the Nano/power supply.
 *
 * I disabled the power LED on the Nano (by breaking it with a knife) 
 * because my enclosure is not 100% lightproof, and the power LED is 
 * quite bright. 
 *
 * The rotary encoder is an incremental mechanical encoder.  This means it
 * has two switches that toggle between on and off in an offset cycle as 
 * the encoder is rotated.  With the internal pull-up resistors activated, 
 * this means that as the encoder is turned, its pins read 11, then 10, 
 * then 00, then 01, and then back to 11 and so on.  My encoder has 
 * detents that "click" the encoder into place at the 11 positions, of 
 * which there are 20 per 360 degree rotation. Some rotary encoders have a 
 * detent at both 00 and 11 in the quadrature encoding cycle.  For these, 
 * you likely would want to double the number of patterns you look for 
 * in the code below, to capture all of the detents.
 *
 */

// If you change pinA, pinB and/or pinC, make sure to adjust the 
// register reads accordingly in loop().
const static int pinA = 2;    // encoder pin A
const static int pinB = 3;    // encoder pin B
const static int pinC = 4;    // pushbutton pin
const static int pinL = 5;    // lights (via MOSFET)

void setup() {
  // encoder inputs, using internal pull-up resistors
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP); 
  pinMode(pinC, INPUT_PULLUP); 
  
  // output for our light, which we will control with PWM
  pinMode(pinL, OUTPUT);

  // for debugging purposes
  // if removed, will speed up the timing of the loop, 
  // which may introduce bounce issues
  Serial.begin(115200);
  
} /* end setup() */

void loop(){
  
  static byte history = B1111;  // encoder - only lower nibble used
  static byte dimmer = 19;      // full power when first on
  static byte switchState = 0;  // is button down or up?
  static byte powerState = 0;   // is power on or off?
  static byte cycleCount = 0;   // used to debounce button

  /*
   * We use a lookup table for dimming values, because perception of
   * brightness does not vary linearly with luminance.  Instead, according
   * to a 1931 study by the International Commission on Illumination 
   * (abbreviated CIE, from the French, "Commission Internationale de 
   * l´Eclairage"), there are two formulas that can be used to model the
   * situation - one with a shallow slope for the bottom 8 percentile, and 
   * a cubic equation that covers the rest of the curve. The 18% gray card
   * used by photographers is due to this study (18% luminance ~= 50%
   * perceived brightness).  For L* (perceived brightness, ranging from 0 
   * to 100) and a white value of Yn, we calculate Y (our duty cycle) as:
   * 
   *   for L* <= 8, 
   *     Y = L* / (8 / (6 / 29)^3 * Yn
   *
   *   for L* > 8, 
   *     Y = (L* + 16) / 116 * Yn
   *
   * 20 dimming values allows all dimming values to readily be reached
   * in a single "spin" with my rotary encoder.
   *
   * With 8-bit PWM, everything except the 5% (lowest) dimming value is
   * within 5% of the calculated CIE 1931 value.  The 5% value is too
   * low by 29%, but this is not too bad.  If 16-bit PWM is used, the
   * accuracy can be substantially improved, but in real life the
   * difference is not noticeable. 
   *
   */
  static byte dimState[] = {1,3,5,8,11,16,22,29,37,47,
                            58,72,87,104,123,145,168,195,223,255};
  
  /* 
   * (1) Check for light on/off (binary light switch).
   * 
   * If the button is LOW, switch power state, and then ignore
   * switch until it is HIGH for 2 cycles through the loop. 
   *
   */
   
  // If waiting for pushbutton press...
  if (! switchState) {      
   
    // Check if pushbutton is pressed, and process if so.
    if ( ! (PIND & 1 << PIND4 ) ) {    // digitalRead(D4);
      switchState = 1;                 // switch to debounce mode
      powerState ^= 1;                 // toggle power state 
      if (powerState) {
        Serial.print("Power on: ");
        Serial.println(dimmer);
        analogWrite(pinL, dimState[dimmer]);
      }
      else {
        Serial.println("Power off.");
        analogWrite(pinL, 0);
      }
    }
  }
  
  // If debouncing after a pushbutton press...
  else {
    // Wait until pushbutton settles at button HIGH (off) for 2 
    // cycles, which is enough to debounce due to the loop delay.
    if  (! (PIND & 1 << PIND4) )       // digitalRead(D4);
      cycleCount = 0;
    else if ( ++cycleCount == 2 ) 
      switchState = cycleCount = 0;
  }

  /* (2) Read rotary encoder
   * 
   * We use the registers to allow reading of both pins together. 
   * We keep track of the current reading and the immediately prior reading 
   * this allows us to detect valid sequences, avoiding most bounce 
   * noise.
   *
   */
  history >>= 2;            // store old state in bits 1 & 2
  history |= PIND & B1100;  // current state is stored in bits 3 & 4
  
  /* (3) Adjust dimming value if necessary
   * 
   * If light is on and the encoder has been rotated, adjust dimming 
   * our rotary dimmer has detents once every four state changes - i.e. 
   * the detents occur after full quadtrature encoding cycles.  We 
   * therefore only look for the state changes that happen when a new 
   * detent position is reached.  Because the two pins are HIGH at the 
   * detents, this means we look for 
   *   B1110 for CW turns, and 
   *   B1101 for CCW turns.
   *
   */
  if (powerState) { 

    // If there is a CW turn to detent position...
    if ( history == B1110 && dimmer < 19) {
      dimmer++;
      Serial.print("Dimmer position: ");
      Serial.println(dimmer);
    }

    // If there is a CCW turn to detent position...
    else if ( history == B1101 && dimmer ) {
      dimmer--;
      Serial.print("Dimmer position: ");
      Serial.println(dimmer);
    }
    
    // We do this here to keep the loop timing the same even if we are
    // not incrementing/decrementing - otherwise we can get some switch
    // bounce at the top/bottom of the range; analogWrite takes on the
    // order of 200 ms.
    analogWrite(pinL, dimState[dimmer]);
  }
  
  delayMicroseconds(500); // debounce delay

} /* end loop() */
