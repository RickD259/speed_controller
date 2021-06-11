//arduino pro mini 5V
//Code version 0.2
//Motor PWM
//Added NeoPixel code
//
//millis() Number of milliseconds passed since the program started
//
/* 
                +--------------------------------+
                |  [GND][GND][VCC][RX][TX][DTR]  |
                |              FTDI              |
                | [ ]1/TX                 RAW[ ] |    
                | [ ]0/RX                 GND[ ] |    
                | [ ]RST        SCL/A5[ ] RST[ ] |   
                | [ ]GND        SDA/A4[ ] VCC[ ] |    
                | [ ]2/INT0    ___      A3/17[ ] |  
Motor PWM       |~[ ]3/INT1   /   \     A2/16[ ] |    
Button input 1  | [ ]4       /PRO  \    A1/15[ ] |  Battery Voltage
Brake switch    |~[ ]5       \ MINI/    A0/14[ ] |  Throttle Signal
Motor PWM (alt) |~[ ]6        \___/    SCK/13[ ] |  led built in
NeoPixel        | [ ]7          A7[ ] MISO/12[ ] |  
Button input 2  | [ ]8          A6[ ] MOSI/11[ ]~|  
                |~[ ]9                  SS/10[ ]~|  
                |           [RST-PB]             |    
                +--------------------------------+  
*/
//J1 throttle, J2 brake, J3 NeoPixel, J4 Button 1, J5 Button 2
// throttle wires (red = +5v, Yellow = gnd, white = signal)
// add way to configure throttle ranges with buttons / on boot
// add thermister?
// add current sense

//----------------neopixel stuff----------
#include <Adafruit_NeoPixel.h>

#define PIN        7 // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS 16 // How many NeoPixels are attached to the Arduino?

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//---------------end neopixel--------------


// These constants won't change. They're used to give names to the pins used:
const int batt_volatge_pin = A1;  // Analog input pin
const int throttle_pin = A0;     // Analog input pin 
const int motor_pwm_pin = 3;      //pwm output (alternative can be 6)
const int brake_pin = 5;         // brake input pin pulled high
const int button1_pin = 4;         // button input pin pulled high
const int button2_pin = 8;         // button input pin pulled high
//const int neopixel_pin = 6;       // ? 


// Adjust these based on hardware setup
int batt_cell_sel = 6;        // number of lipo cells. 10 MAX. 2cell 8.4V, 3cell 12.6V, 4cell 16.8V, 5cell 21.0V, 6cell 25.2V, 7cell 29.4V, 8cell 33.6V, 9cell 37.8V, 10cell 42.0V,
int batt_cell_v = 3200;        // battery cell min value. lipo = 3.2v ( x 1000 )
int batt_cell_max_v = 4200;        // battery cell max value. lipo = 4.2v ( x 1000 )

int throttle_max = 850;      //adc value of 100% throttle (850 = 4.15V)
int throttle_min = 285;        //adc value of 0% throttle (285 = 1.39v)

//variables
int throttle_input = 0;       // throttle position, 0 to 1023
int batt_v_input = 0;         // battery pack voltage, 0 to 1023, shut off motor when to low
int motor_pwm = 0;            // actual motor pwm output 0 to 255
int command_pwm = 0;          // requested pwm output 0 to 255
int brake_state;
int button1_state;
int button2_state;

unsigned int batt_v_min = 0;           // battery value where motor will shut off
unsigned int batt_v = 0;               // battery voltage (converted from adc)
byte batt_low = 0;            // bat low flag
byte batt_ex_low = 0;         // bat very low flag

int ramp = 0;           //pwm ramp up speed

int pixel_number = 0; //neo pixel stuff
int pixel_max = 0; //
int neo_green = 0; //neo pixel stuff
int neo_red = 0; //neo pixel stuff
int neo_blue = 0; //neo pixel stuff

unsigned long currentMillis;
unsigned long battMillis = 0; 
unsigned long previousMillis = 0;
unsigned long testMillis = 0;      //using for print out


void setup() {
  //Code for Available PWM frequency for D3 & D11: uncomment one
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011; // for PWM frequency of 980.39 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100; // for PWM frequency of 490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101; // for PWM frequency of 245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110; // for PWM frequency of 122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz

  //Code for Available PWM frequency for D5 & D6: uncomment one
  //TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000011; // for PWM frequency of 976.56 Hz (The DEFAULT)
  //TCCR0B = TCCR0B & B11111000 | B00000100; // for PWM frequency of 244.14 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000101; // for PWM frequency of 61.04 Hz

  //Code for Available PWM frequency for D9 & D10: uncomment one
  //TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011; // for PWM frequency of 490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
  //Serial.begin(115200); // initialize serial communications at  bps:
  
  pinMode(brake_pin, INPUT_PULLUP);
  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {//==========================================================================

  currentMillis = millis(); //time since program started

  


  
  check_batt();
  check_throttle();
  check_buttons();
  neopixel_stuff();
  
  if (brake_state == HIGH && batt_ex_low == 0){
    analogWrite(motor_pwm_pin, motor_pwm);
  } else {
    digitalWrite(motor_pwm_pin, LOW);
    motor_pwm = 0;                    // need this to work with the slow ramping up speed function in check_throttle
  }





  
 

/*
// print the results to the Serial Monitor:
if (currentMillis - testMillis > 100){
  Serial.print("bat v = ");
  Serial.print(batt_v);
  Serial.print(" bat v min = ");
  Serial.print(batt_v_min);
  Serial.print(" bat an = ");
  Serial.println(batt_v_input); 

  Serial.print("m pwm = ");
  Serial.print(motor_pwm);
  Serial.print(" c pwm = ");
  Serial.print(command_pwm);
  Serial.print(" throttle = ");
  Serial.println(throttle_input);
  testMillis = currentMillis;
}
*/ 




}//==========================================================================



void check_batt() {
  // read the analog input value:
  batt_v_input = analogRead(batt_volatge_pin);
  
  //calculate the min safe battery voltage before shutting down the motor.
  batt_v_min = batt_cell_sel * batt_cell_v; // result in voltage x 1000

  //voltage divider
  // 110 / 10 * (2.1212) = 23.33v   |   5 / 1023 * [434] = 2.1212
  //batt_v = (110 / 10 * (1023 / 5 * batt_v_input)) * 100; //result in voltage x 100
  batt_v = (4.887 * batt_v_input) * 11; //result in voltage x 1000


  //change to trigger batt_low at 3300 (3.3V) and batt_ex_low at 3200 (3.2V)?
  if (batt_v < batt_v_min){
    digitalWrite(LED_BUILTIN, HIGH); 
    if (currentMillis - battMillis > 5000){  //if bat low for 5 secs in a row trigger vey low flag
      batt_ex_low = 1;
    }
  } else if (batt_v < batt_v_min + 1000){
    digitalWrite(LED_BUILTIN, HIGH);
    if (currentMillis - battMillis > 5000){  //if bat low for 5 secs in a row trigger low flag
      batt_low = 1;
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    battMillis = currentMillis;
  }


  // 1023 / 5.0v * [2.12v] = 434
  // 5 / 1023 * [434] = 2.1212

  // batt_v_input = 0 to 1023 (10 bits), 0v to 5v
  // volatge divider on pcb = 100K & 10K
  // analog v = bat v * 10 / 110 (4.545v = 50v)
  // 110 / 10 * [4.545v] = 49.99v
  // if batt_v_input < batt_v_min then shutdown


  
}//==========================================================================


void check_throttle() {
  // read the analog input value:
  throttle_input = analogRead(throttle_pin);
  
  // map it to the range of the analog out:
  command_pwm = map(throttle_input, throttle_min, throttle_max, 0, 255);    // change sensorValue (0 to 1023) to outputValue (0 to 255)
  command_pwm = constrain(command_pwm, 0, 255); // needed to keep motor pwm between 0 and 255 due to throttle_min/max

  // low pwm range boost.
  if (command_pwm > 0 && command_pwm < 20) {
    command_pwm = 20;   
  }

  // battery low check, limit max speed
  if (batt_low == 1 && command_pwm > 150) {
    command_pwm = 150;   
  }

/*
  //change ramp up speed based on current speed
  if (motor_pwm > 80){
    ramp = 3;
  } else if (motor_pwm > 60) {
    ramp = 5;
  } else {
    ramp = 10;
  }
*/

  //maybe use this?
  ramp = map(motor_pwm, 20, 200, 10, 1);
  ramp = constrain(ramp, 1, 10); // needed to pwm between 0 and 255


  // slow the ramping up speed of the motor to protect it
  if (motor_pwm > command_pwm) {
    motor_pwm = command_pwm;
  } else {
    if (currentMillis - previousMillis > ramp) {
      previousMillis = currentMillis;
      if (command_pwm > motor_pwm){                
        motor_pwm = motor_pwm + 1; 
        if (motor_pwm > command_pwm) {
          motor_pwm = command_pwm;
        }      
      }
    } 
  }



  
}//==========================================================================


void check_buttons() {
  //int reading = digitalRead(button_pin);
  brake_state = digitalRead(brake_pin);
  button1_state = digitalRead(button1_pin);
  button2_state = digitalRead(button2_pin);
  /*
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (brake_state != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 10) {
    if (brake_state != buttonState) {
      buttonState = brake_state;
      if (buttonState == LOW) {
        mode = mode + 1;
        new_mode_flag = HIGH;
      }
    }
  }



  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = brake_state;
  */
    
}//==============================



void neopixel_stuff() {

int batt_full_scale = batt_cell_sel * batt_cell_max_v - batt_v_min;
int batt_curr_level = batt_v - batt_v_min;

// show battery level when throttle at 0 (batt_v - batt_v_min = current level) (batt_cell_sel * 1000 = full scale)
// show throttle level when used
// red when brake
// throttle level orange when bat low
// flash orange when dead

//batt_low == 1;
//batt_ex_low == 1;

 //command_pwm
  if (brake_state == HIGH){
    if (motor_pwm > 20){
      if (batt_low == 1){
        neo_red = 200;
        neo_green = 200;
        neo_blue = 0;        
      } else {
        neo_red = 200;
        neo_green = 200;
        neo_blue = 200;
      }
      pixel_max = map(motor_pwm, 0, 255, 0, NUMPIXELS); // pwm min, pwm max, pixel min, pixel max)
      pixel_max = constrain(pixel_max, 0, NUMPIXELS);  
    } else {
      neo_red = 0;
      neo_green = 100;
      neo_blue = 0;
      pixel_max = map(batt_curr_level, 0, batt_full_scale, 0, NUMPIXELS);
      pixel_max = constrain(pixel_max, 0, NUMPIXELS);  
    }
 
  } else {
    neo_red = 255;
    neo_green = 0;
    neo_blue = 0; 
    pixel_max = 16;                   
  }


  pixel_number = pixel_number + 1;
  if (pixel_number > NUMPIXELS) pixel_number=0;

  if (pixel_number > pixel_max){ // set unused pixels to off or w/e
    neo_red = 2;
    neo_green = 2;
    neo_blue = 2; 
  }

  
  pixels.setPixelColor(pixel_number, pixels.Color(neo_red, neo_green, neo_blue));  //(r, g, b) 0 to 255 
  pixels.show();   // Send the updated pixel colors to the hardware.

  
}//========================================================================
