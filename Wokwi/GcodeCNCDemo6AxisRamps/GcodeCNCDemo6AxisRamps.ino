//------------------------------------------------------------------------------
// 6 Axis CNC Demo RAMPS - supports RAMPS 1.4 controller
// dan@marginallyclever.com 2013-10-28
// DaveX 202402022
// RAMPS should be treated like a MEGA 2560 Arduino.
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

// https://wokwi.com/projects/390478474763185153
// Modified from RUMBA Code & simulation at https://wokwi.com/projects/327981866411885138
// Modified from https://github.com/MarginallyClever/GcodeCNCDemo/blob/master/GcodeCNCDemo6AxisRumba/GcodeCNCDemo6AxisRumba.ino
// for Wokwi Mega with Stepper drivers
// 
// Also includes pull request https://github.com/MarginallyClever/GcodeCNCDemo/pull/36
// enabling feed rates slower than 200 steps/sec.
//
//  See also: https://wokwi.com/projects/390741120778917889 
//  for a 6-axis coordinated control without gcode, but with Bresenham.
//
//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.

#define VERSION              (2)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MAX_FEEDRATE         (1000000.0) // steps/sec
#define MIN_FEEDRATE         (0.01)  // steps/sec
#define NUM_AXIES            (6)

// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];

char serialBuffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr=0;  // human version
long step_delay;  // machine version

float px,py,pz,pu,pv,pw;  // position

// settings
char mode_abs=1;  // absolute mode?

long line_number=0;


// RAMPS pins from https://reprap.org/wiki/RAMPS_1.4#Pins & https://forum.arduino.cc/t/arduino-mega-use-analog-pins-as-digital/13844/6?u=davex
// For RAMPS 1.4
#define X_STEP_PIN         54 //A0 
#define X_DIR_PIN          55 //A1
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN          -1 //PIN 2 is used

#define Y_STEP_PIN         60 //A6
#define Y_DIR_PIN          61 //A7
#define Y_ENABLE_PIN       56 //A2
#define Y_MIN_PIN          14
#define Y_MAX_PIN          -1 //PIN 15 is used

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          -1 //PIN 19 is used

//extruder 1
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

//extruder 2
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30


#define SDPOWER            -1

//ChipSelect, Hardware SS Pin on Mega, 10 for Arduino Boards, always kept as output
#define SDCS_PIN           53
#define SD_DETECT_PIN 	   -1 //currently not implemented


#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12	//ATX , awake=LOW, SLEEP=High
#define KILL_PIN           -1

#define HEATER_0_PIN	10  // Extruder Heater
#define HEATER_1_PIN	8

#define TEMP_0_PIN		13   // ANALOG NUMBERING
#define TEMP_1_PIN		14   // ANALOG NUMBERING

// extra on end
#define AUX_STEP_PIN  16
#define AUX_DIR_PIN   17
#define AUX_ENABLE_PIN 23
#define AUX_MIN_PIN 25
#define E0_MIN_PIN 27
#define E1_MIN_PIN 29



//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) { // 
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = MAX_FEEDRATE/nfr;
  //Serial.print("step_delay");
  //Serial.println(step_delay);

  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy,float npz,float npu,float npv,float npw) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
  pu=npu;
  pv=npv;
  pw=npw;
}


/**
 * Supports movement with both styles of Motor Shield
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void onestep(int motor) {
#ifdef VERBOSE
  char *letter="XYZUVW";
  Serial.print(letter[motor]);
#endif
  
  digitalWrite(motors[motor].step_pin,HIGH);
  digitalWrite(motors[motor].step_pin,LOW);
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy,float newz,float newu,float newv,float neww) {
  a[0].delta = newx-px;
  a[1].delta = newy-py;
  a[2].delta = newz-pz;
  a[3].delta = newu-pu;
  a[4].delta = newv-pv;
  a[5].delta = neww-pw;
  
  long i,j,maxsteps=0;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].absdelta = abs(a[i].delta);
    if( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    // set the direction once per movement
    digitalWrite(motors[i].dir_pin,a[i].delta>0?HIGH:LOW);
  }
  for(i=0;i<NUM_AXIES;++i) {
    a[i].over=maxsteps/2;
  }
  
  long dt = MAX_FEEDRATE / 5000; // sets starting speed
  long accel = 1;  // us/step
  long steps_to_accel = dt - step_delay;
  if(steps_to_accel < 0){
    steps_to_accel = 0;
    dt = step_delay;
  }
  if(steps_to_accel > maxsteps/2 ) 
    steps_to_accel = maxsteps/2;
    
  long steps_to_decel = maxsteps - steps_to_accel;

  Serial.print("START ");
  Serial.println(dt);
  Serial.print("TOP ");
  Serial.println(step_delay);
  
  Serial.print("accel until ");
  Serial.println(steps_to_accel);  
  Serial.print("decel after ");
  Serial.println(steps_to_decel);  
  Serial.print("total ");
  Serial.println(maxsteps);  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  for( i=0; i<maxsteps; ++i ) {
    for(j=0;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps) {
        a[j].over -= maxsteps;
        
        digitalWrite(motors[j].step_pin,HIGH);
        digitalWrite(motors[j].step_pin,LOW);
      }
    }

    if(i<steps_to_accel) {
      dt -= accel;
    }
    if(i>=steps_to_decel) {
      dt += accel;
    }
    pause(dt);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  position(newx,newy,newz,newu,newv,neww);
}


// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
static void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny,pz,pu,pv,pw);
  }
  
  line(x,y,pz,pu,pv,pw);
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=serialBuffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)serialBuffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("Z",pz);
  output("U",pu);
  output("V",pv);
  output("W",pw);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo6AxisV2 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X/Y/Z/U/V/W(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/U/V/W(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
  Serial.println(F("Try G1 X100 Y200 Z300 U400 V500 W600 F60"));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parseNumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parseNumber('F',fr));
    line( parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          parseNumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz),
          parseNumber('U',(mode_abs?pu:0)) + (mode_abs?0:pu),
          parseNumber('V',(mode_abs?pv:0)) + (mode_abs?0:pv),
          parseNumber('W',(mode_abs?pw:0)) + (mode_abs?0:pw) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parseNumber('F',fr));
      arc(parseNumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parseNumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0),
              parseNumber('Y',0),
              parseNumber('Z',0),
              parseNumber('U',0),
              parseNumber('V',0),
              parseNumber('W',0) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1);
  switch(cmd) {
  case 17:  motor_enable();  break;
  case 18:  motor_disable();  break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * set up the pins for each motor
 */
void motor_setup() {
  motors[0].step_pin=X_STEP_PIN;
  motors[0].dir_pin=X_DIR_PIN;
  motors[0].enable_pin=X_ENABLE_PIN;    
  motors[0].limit_switch_pin=X_MIN_PIN;

  motors[1].step_pin=Y_STEP_PIN;
  motors[1].dir_pin=Y_DIR_PIN;    
  motors[1].enable_pin=Y_ENABLE_PIN;
  motors[1].limit_switch_pin=Y_MIN_PIN;

  motors[2].step_pin=Z_STEP_PIN;
  motors[2].dir_pin=Z_DIR_PIN;    // 56/A2 on Rumba
  motors[2].enable_pin=Z_ENABLE_PIN;  // 62/A8 on Rumba
  motors[2].limit_switch_pin=Z_MIN_PIN;

  motors[3].step_pin=E0_STEP_PIN;
  motors[3].dir_pin=E0_DIR_PIN;
  motors[3].enable_pin=E0_ENABLE_PIN;
  motors[3].limit_switch_pin=Z_MIN_PIN;

  motors[4].step_pin=E1_STEP_PIN;
  motors[4].dir_pin=E1_DIR_PIN;
  motors[4].enable_pin=E1_ENABLE_PIN;
  motors[4].limit_switch_pin=E1_MIN_PIN;

  motors[5].step_pin=AUX_STEP_PIN;
  motors[5].dir_pin=AUX_DIR_PIN;
  motors[5].enable_pin=AUX_ENABLE_PIN;
  motors[5].limit_switch_pin=AUX_MIN_PIN;
  
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
  }
}


void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,LOW);
  }
}


void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,HIGH);
  }
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  motor_setup();
  motor_enable();
  
  help();  // say hello
  position(0,0,0,0,0,0);  // set staring position
  feedrate(1000);  // set default speed
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) serialBuffer[sofar++]=c;  // store it
    if(c=='\n') {
      // entire message received
      serialBuffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/


