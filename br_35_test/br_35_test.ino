#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define RTC_ADDR 0b1101000 // rtc i2c address
#define HALL_INT 2 // hall effect sensor pin for interrupts
#define ERR_LED 3 // error indicator pin
#define EXTEND_RELAY 5 // piston extend relay pin
#define RETRACT_RELAY 6 // piston retract relay pin
#define MACHINE_CTRL 7 // machine control pin (high: machine runs)
#define RETRACT_ENDSTOP 8 // piston endstop retracted position pin
#define EXTEND_ENDSTOP 9 // piston endstop end position pin
#define OVERHEAT_REST_INTERVAL 300000 // 5mins
#define THERM_PIN A1 // thermistor pin
#define CURR_PIN A3 // current sensor pin
#define TEMP_LIMIT 100 // max motor temp
#define CURR_LIMIT 240 // max current (dA)
#define RPM_LIMIT 500 // min brush rpm on free rotation
#define INTERVAL 190 // INTERVAL for measurements
#define CURR_CALIBRATION 508 // value for centering adc value
#define CYCLE_MAX 10000 // number of cycles to run

struct time_buf {
  uint8_t secs;
  uint8_t mins;
  uint8_t hrs;
  uint8_t day;
  uint8_t mnth;
};

void record_to_sd(uint8_t temp, uint16_t rpm, uint16_t current, uint16_t cycle); // record data to sd
void read_rtc(time_buf* buf); // read time
void retract(uint8_t* piston_status); // retract cylinder routine
void extend(uint8_t* piston_status); // extend cylinder routine
void blink_err_led(uint16_t off_period, uint16_t on_period); // error led blink helper function
void error(uint8_t code); // error routine (0 overheat, 1 overcurrent, 2 low rpm, 3 sd card, 4 piston failure)
uint8_t bcd_to_dec(uint8_t a); // convert bcd to dec
uint8_t init_piston(uint8_t* piston_status); // initialize piston position
uint8_t read_temp(uint8_t adc_ch); // read temperature from thermistor on adc channel
uint16_t read_current(uint8_t adc_ch); // read current from sensor on adc channel
uint16_t read_rpm(uint8_t cntrl); // read rpm of brush (0 to start, 1 to read)

// thermistor lookup R1=9.95K Rt=100k@25C
uint16_t temp_lookup[][2] = { 
  {975, 10}, // {ADC value, temperature}
  {963, 15},
  {948, 20},
  {930, 25},
  {911, 30},
  {888, 35},
  {863, 40},
  {834, 45},
  {801, 50},
  {768, 55},
  {732, 60},
  {693, 65},
  {653, 70},
  {611, 75},
  {570, 80},
  {529, 85},
  {489, 90},
  {449, 95},
  {412, 100},
  {379, 105},
  {345, 110},
  {314, 115},
  {285, 120},
  {257, 125}
};

uint8_t interval_iter = 0; // interval iterator
uint8_t temp_buffer = 0; // temp buffer
uint8_t piston_status = 0; // piston status flag (0 retracted, 1 extended)
uint16_t current_buffer = 0; // current buffer
uint16_t rpm_buffer = 0; // rpm buffer
uint16_t cycle_iter = 0; // number of cycles passed
uint16_t rpm_iter = 0; // rpm iterator
uint32_t prev_millis = 0; // timing variable
uint32_t curr_millis = 0; // timing variable

void setup() {
  // initialize combusses
//  Serial.begin(9600); // open serial port for debugging
  Wire.begin(); // start i2c bus

  // initialize outputs
  pinMode(MACHINE_CTRL, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(EXTEND_RELAY, OUTPUT);
  pinMode(RETRACT_RELAY, OUTPUT);
  digitalWrite(MACHINE_CTRL, 0);
  digitalWrite(ERR_LED, 0);
  digitalWrite(EXTEND_RELAY, 0);
  digitalWrite(RETRACT_RELAY, 0);

  // initialize inputs
  pinMode(RETRACT_ENDSTOP, INPUT);
  pinMode(EXTEND_ENDSTOP, INPUT);

  // initialize sd card
  if(!SD.begin()) error(3); 

  // initialize piston to retracted position
  if(!init_piston(&piston_status)) error(4);

  delay(5000);
}

void loop() {
  // check time
  curr_millis = millis();

  if(curr_millis - prev_millis >= INTERVAL) { // if INTERVAL reached (every .2s)
    // record time
    prev_millis = curr_millis;

    if(cycle_iter < CYCLE_MAX) { // if havent finished
      // measure temp
      temp_buffer = read_temp(THERM_PIN);
      
      // if overheating, rest
      while(temp_buffer >= TEMP_LIMIT) {
        digitalWrite(MACHINE_CTRL, 0); // turn off machine
        retract(&piston_status); // retract
        record_to_sd(temp_buffer, 0, 0, cycle_iter);
        error(0);
        delay(OVERHEAT_REST_INTERVAL);
        temp_buffer = read_temp(THERM_PIN);
        record_to_sd(temp_buffer, 0, 0, cycle_iter);
        interval_iter = 0;
      }

      // check if running, start if not
      digitalRead(MACHINE_CTRL) ? void() : digitalWrite(MACHINE_CTRL, 1);
      
      // control machine
      ++interval_iter <= 40 ? 
        retract(&piston_status) : // 8s/10s
        extend(&piston_status); // 2s/10s

      // start rpm measurement
      if(interval_iter == 5 || interval_iter == 48) {
        read_rpm(0); // start rpm measurement
      }
  
      // record and check info every 5s
      if(interval_iter == 26 || interval_iter == 52) {
        // record
        current_buffer = read_current(CURR_PIN); // record current
        rpm_buffer = read_rpm(1); // record rpm
        record_to_sd(temp_buffer, rpm_buffer, current_buffer, cycle_iter);

        // check variables
        if(current_buffer >= CURR_LIMIT) error(1);
        if(interval_iter <= 40 && rpm_buffer <= RPM_LIMIT) error(2); // check brush speed on free rotation
      }
  
      if(interval_iter == 52) { // every 10s
        interval_iter = 0; // reset iteration
        cycle_iter++; // iterate cycles
      }
    } else {
      digitalWrite(MACHINE_CTRL, 0); // if finished, turn machine off
      retract(&piston_status); // retract machine
    }
  }

//  // ENDSTOP CALIBRATION (TODO: make a separate function)
//  digitalWrite(MACHINE_CTRL, 1);
//  digitalWrite(EXTEND_RELAY, 1);
//  while(read_current(CURR_PIN) <= 180) {
//    delayMicroseconds(50);
//  }
//  digitalWrite(EXTEND_RELAY, 0);
//  delay(2000);
//  digitalWrite(MACHINE_CTRL, 0);
//  while(1);

//  // SANITY CHECKS
//  digitalWrite(MACHINE_CTRL, 1);
//  read_rpm(0);
//  delay(5000);
//  Serial.print(read_temp(THERM_PIN));
//  Serial.print("\t");
//  Serial.print(read_current(CURR_PIN));
//  Serial.print("\t");
//  Serial.println(read_rpm(1));
//  digitalWrite(MACHINE_CTRL, 0);
//  delay(2000);
//  extend(&piston_status);
//  delay(5000);
//  retract(&piston_status);
//  delay(5000);
//  delay(1000);
//  record_to_sd(5, 10, 15);
}

void record_to_sd(uint8_t temp, uint16_t rpm, uint16_t current, uint16_t cycle) {
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
    data_file.print(temp);
    data_file.print("\t");
    data_file.print(rpm);
    data_file.print("\t");
    data_file.println(current);
    data_file.close();
  } else error(3);
}

uint8_t bcd_to_dec(uint8_t a) {
  return(a - 6 * (a >> 4));
}

uint8_t init_piston(uint8_t* piston_status) {
  uint32_t i = 0; // iterator
  uint32_t timeout_step = 500; // step in us for timeout (0.5ms)
  uint32_t timeout_iters = 4000; // timeout period in timeout_steps (2s)
  
  digitalWrite(RETRACT_RELAY, 1); // try to retract
  while(digitalRead(RETRACT_ENDSTOP)) { // wait to reach endstop
    i++;
    delayMicroseconds(timeout_step);
    
    if(i >= timeout_iters) { // if does not reach endstop in timeout_iters
      i = 0; // reset iterator
      digitalWrite(RETRACT_RELAY, 0); // turn off retract relay
      delay(20);
      digitalWrite(EXTEND_RELAY, 1); // extend
      
      while(digitalRead(RETRACT_ENDSTOP)) { // wait to reach endstop
        i++;
        delayMicroseconds(timeout_step);
        if(i >= timeout_iters) return(0); // something wrong
      }
      
      digitalWrite(EXTEND_RELAY, 0); // turn off extend relay
    }
  }

  digitalWrite(RETRACT_RELAY, 0);
  *piston_status = 0;
  
  return(1);
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

void retract(uint8_t* piston_status) {
  uint32_t i = 0; // iterator
  uint32_t timeout_step = 200; // step in us for timeout (0.2ms)
  uint32_t timeout_iters = 15000; // timeout period in timeout_steps (3s)
  
  if(*piston_status) {
    digitalWrite(RETRACT_RELAY, 1);
    while(digitalRead(RETRACT_ENDSTOP)) {
      i++;
      delayMicroseconds(timeout_step);
      if(i >= timeout_iters) error(4); // if doesnt retract in timout period
    }
    digitalWrite(RETRACT_RELAY, 0);
    *piston_status = 0; // set flag
  }
}

void extend(uint8_t* piston_status) {
  uint32_t i = 0; // iterator
  uint32_t timeout_step = 200; // step in us for timeout (0.2ms)
  uint32_t timeout_iters = 15000; // timeout period in timeout_steps (3s)

  if(!(*piston_status)) {
    digitalWrite(EXTEND_RELAY, 1);
    while(digitalRead(EXTEND_ENDSTOP)) {
      i++;
      delayMicroseconds(timeout_step);
      if(i >= timeout_iters) error(4); // if doesnt extend in timeout period
    }
    digitalWrite(EXTEND_RELAY, 0);
    *piston_status = 1; // set flag
  }
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
    case 1: // overcurrent
      digitalWrite(MACHINE_CTRL, 0);
      retract(&piston_status);
      blink_err_led(500, 500);
      break;
    case 2: // low rpm
      digitalWrite(MACHINE_CTRL, 0);
      retract(&piston_status);
      blink_err_led(2000, 500);
      break;
    case 3: // sd card
      digitalWrite(MACHINE_CTRL, 0);
      retract(&piston_status);
      blink_err_led(500, 2000);
      break;
    case 4: // piston failure
      digitalWrite(MACHINE_CTRL, 0);
      blink_err_led(2000, 2000);
      break;
  }
}

uint8_t read_temp(uint8_t adc_ch) {
  uint16_t adc_val;
  uint8_t i = 0;
  uint16_t adc_val0, adc_val1; // values for linear interpolation
  uint8_t temp0, temp1; // values for linear interpolation
  
  adc_val = analogRead(adc_ch);

  // search for adc_value
  while(adc_val < temp_lookup[i][0]) {
    i++;
  }

  // lookup values
  if(i) {
    adc_val0 = temp_lookup[i-1][0];
    adc_val1 = temp_lookup[i][0];
    temp0 = temp_lookup[i-1][1];
    temp1 = temp_lookup[i][1];
  } else return(0);

  // return interpolated temperature
  return((temp0 * (adc_val - adc_val1) + temp1 * (adc_val0 - adc_val)) / 
         (adc_val0 - adc_val1));
}

uint16_t read_rpm(uint8_t cntrl) {
  static uint32_t local_prev_millis = 0; // local millis buffer
  static uint32_t local_curr_millis = 0; // local millis buffer
  
  if(!cntrl) {
    rpm_iter = 0; // reset iterator
    
    local_curr_millis = millis();
    
    attachInterrupt(digitalPinToInterrupt(HALL_INT), hall_interrupt, FALLING); // reattach interrupt    
    
    return(1);
  } else {
    detachInterrupt(digitalPinToInterrupt(HALL_INT)); // detach interrupt

    // record time
    local_prev_millis = local_curr_millis;
    local_curr_millis = millis();
  
    return(rpm_iter * (60000 / (local_curr_millis - local_prev_millis))); // calc rpm
  }
}

uint16_t read_current(uint8_t adc_ch) {
  int16_t adc_val;
  
  adc_val = analogRead(adc_ch); // read adc
  adc_val = abs(adc_val - CURR_CALIBRATION); // center value
  
  // return current (sensitivity: 100mV / 1A; 2.5V@0A (centered))
  return(adc_val / 2);
}

void hall_interrupt(void) {
  rpm_iter++; // increment iterator
}

