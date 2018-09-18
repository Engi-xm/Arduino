#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>

#define RTC_ADDR 0b1101000 // rtc i2c address
#define ERR_LED 3 // error indicator pin
//#define EXTEND_RELAY 5 // piston extend relay pin
//#define RETRACT_RELAY 6 // piston retract relay pin
//#define EXTEND_ENDSTOP 7 // piston endstop end position pin
//#define RETRACT_ENDSTOP 8 // piston endstop retracted position pin
#define MACHINE_CTRL 9 // servo signal pin for machine control
#define SERVO_OFF_PWM 25 // servo angle for machine off
#define SERVO_ON_PWM 70 // servo angle for machine on
#define THERM_1_PIN A0 // thermistor pin
#define THERM_2_PIN A1 // thermistor pin
#define THERM_3_PIN A2 // thermistor pin
#define TEMP_LIMIT 100 // max motor temp
#define OVERHEAT_REST_INTERVAL 300000 // 5mins
#define INTERVAL 30000 // interval for checks (30s)
#define CYCLE_MAX 10000 // number of cycles to run

struct time_buf { // define time buffer structure
  uint8_t secs;
  uint8_t mins;
  uint8_t hrs;
  uint8_t day;
  uint8_t mnth;
};
struct temp_buf { // define temp buffer structure
  uint8_t t1;
  uint8_t t2;
  uint8_t t3;
};

Servo machine_ctrl; // create servo object

void record_to_sd(temp_buf* temp, uint16_t cycle); // record time, current and temperature
void read_rtc(time_buf* buf); // read time
void blink_err_led(uint16_t off_period, uint16_t on_period); // error led blink helper function
void error(uint8_t code); // error routine (0 overheat, 3 sd card)
uint8_t bcd_to_dec(uint8_t a); // convert bcd to dec
uint8_t read_temp(uint8_t adc_ch); // read temperature from thermistor on adc channel
uint8_t machine_toggle(uint8_t machine_status); // toggle machine on (1)/ off (0)

// thermistor lookup R=10K Rt=100k@25C
uint16_t temp_lookup[][2] = { 
  {974, 10}, // {ADC value, temperature}
  {962, 15},
  {947, 20},
  {930, 25},
  {911, 30},
  {888, 35},
  {862, 40},
  {833, 45},
  {800, 50},
  {767, 55},
  {731, 60},
  {692, 65},
  {652, 70},
  {610, 75},
  {569, 80},
  {528, 85},
  {487, 90},
  {448, 95},
  {411, 100},
  {378, 105},
  {344, 110},
  {313, 115},
  {284, 120},
  {256, 125}
};

uint8_t interval_iter = 0; // interval iterator
uint8_t machine_status = 0; // machine control flag
uint16_t cycle_iter = 0; // number of cycles passed
uint32_t prev_millis = 0; // timing variable
uint32_t curr_millis = 0; // timing variable
temp_buf temp_buffer; // temperature buffer

void setup() {
  // initialize combusses
  Serial.begin(9600); // open serial port for debugging
  Wire.begin(); // start i2c bus

  // initialize servo
  machine_ctrl.attach(MACHINE_CTRL);
  delay(50);
  machine_ctrl.write(SERVO_OFF_PWM);
    
  // initialize outputs
  pinMode(ERR_LED, OUTPUT);
  digitalWrite(ERR_LED, 0);

  // set adc ref to external
  analogReference(EXTERNAL);

  // initialize sd card
  if(!SD.begin()) error(3); 

  delay(1000);
}

void loop() {
  // read temps
  temp_buffer.t1 = read_temp(THERM_1_PIN);
  temp_buffer.t2 = read_temp(THERM_2_PIN);
  temp_buffer.t3 = read_temp(THERM_3_PIN);
  
  // if overheating, rest
  while(temp_buffer.t1 >= TEMP_LIMIT ||
        temp_buffer.t2 >= TEMP_LIMIT ||
        temp_buffer.t3 >= TEMP_LIMIT) {
    machine_status = machine_toggle(0); // turn off machine
    record_to_sd(&temp_buffer, cycle_iter);
    error(0);
    delay(OVERHEAT_REST_INTERVAL);
    temp_buffer.t1 = read_temp(THERM_1_PIN);
    temp_buffer.t2 = read_temp(THERM_2_PIN);
    temp_buffer.t3 = read_temp(THERM_3_PIN);
    record_to_sd(&temp_buffer, cycle_iter);
    interval_iter = 0;
  }
  
  // check time
  curr_millis = millis();

  if(curr_millis - prev_millis >= INTERVAL) { // if INTERVAL reached
    // record time
    prev_millis = curr_millis;

    if(cycle_iter < CYCLE_MAX) { // if havent finished
      // record temps
      record_to_sd(&temp_buffer, cycle_iter);

      // control machine
      interval_iter++ < 10 ?
        machine_toggle(1) : // 5min / 6min
        machine_toggle(0); // 1min / 6min

      if(interval_iter == 12) {
        interval_iter = 0; // reset iteration
        cycle_iter++; // iterate cycles
      }
    } else {
      machine_toggle(0); // if finished, turn machine off
    }
  }
}

void record_to_sd(temp_buf* temp, uint16_t cycle) {
  time_buf time_buffer;

  read_rtc(&time_buffer);
  
  File data_file = SD.open("datalog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (data_file) {
    data_file.print(cycle);
    data_file.print("\t");
    data_file.print(time_buffer.mnth);
    data_file.print('-');
    data_file.print(time_buffer.day);
    data_file.print("\t");
    data_file.print(time_buffer.hrs);
    data_file.print(':');
    data_file.print(time_buffer.mins);
    data_file.print(':');
    data_file.print(time_buffer.secs);
    data_file.print("\t");
    data_file.print(temp->t1);
    data_file.print("\t");
    data_file.print(temp->t2);
    data_file.print("\t");
    data_file.println(temp->t3);
    data_file.close();
  } else error(3);
}

uint8_t bcd_to_dec(uint8_t a) {
  return(a - 6 * (a >> 4));
}

void read_rtc(time_buf* buf) {
  uint8_t loc_buf[6];
  
  // set address to seconds
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(RTC_ADDR, 6); // request information
  
  // read information
  for(uint8_t i = 0; i < 6; i++) {
    loc_buf[i] = Wire.read(); // receive byte
  }

  // copy to buffer
  buf->secs = bcd_to_dec(loc_buf[0]);
  buf->mins = bcd_to_dec(loc_buf[1]);
  buf->hrs = bcd_to_dec(loc_buf[2]);
  buf->day = bcd_to_dec(loc_buf[4]);
  buf->mnth = bcd_to_dec(loc_buf[5]);
}

void blink_err_led(uint16_t off_period, uint16_t on_period) {
  while(1) {
    digitalWrite(ERR_LED, 1);
    delay(on_period);
    digitalWrite(ERR_LED, 0);
    delay(off_period);
  }
}

void error(uint8_t code) {
  switch(code) {
    case 0: // overheat
      digitalWrite(ERR_LED, 1);
      break;
    case 3: // sd card
      machine_toggle(0);
      blink_err_led(500, 2000);
      break;
  }
}

uint8_t read_temp(uint8_t adc_ch) {
  uint16_t adc_val;
  uint8_t i = 0;
  uint16_t adc_val0, adc_val1; // values for linear interpolation
  uint8_t temp0, temp1; // values for linear interpolation

  delay(20); // make sure adc is ready
  
  adc_val = analogRead(adc_ch);

  // search for adc_value
  while(adc_val < temp_lookup[i][0]) {
    if(i++ >= 23) break;
  }

  // lookup values
  if(i > 0  && i < 24) {
    adc_val0 = temp_lookup[i-1][0];
    adc_val1 = temp_lookup[i][0];
    temp0 = temp_lookup[i-1][1];
    temp1 = temp_lookup[i][1];
  } else return(0);

  // return interpolated temperature
  return((temp0 * (adc_val - adc_val1) + temp1 * (adc_val0 - adc_val)) / 
         (adc_val0 - adc_val1));
}

uint8_t machine_toggle(uint8_t machine_status) {
  static uint8_t curr_pos = 0;
  uint8_t k = 9;
  uint8_t pos;
  int8_t travel;
  int8_t unit_step;
  
  if(curr_pos == machine_status) return(machine_status);

  machine_status ? travel = SERVO_ON_PWM - SERVO_OFF_PWM : 
                   travel = SERVO_OFF_PWM - SERVO_ON_PWM; 
  unit_step = travel / k;
  
  for(uint8_t i = 1; i <= k; i++) {
    machine_status ? pos = SERVO_OFF_PWM + unit_step * i :
                     pos = SERVO_ON_PWM + unit_step * i;
    Serial.println(pos);
    machine_ctrl.write(pos);
    delay(50);
  }
  
  curr_pos = machine_status;
  return(machine_status);
}

