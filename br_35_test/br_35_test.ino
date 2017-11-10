#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define RTC_ADDR 0b1101000 // rtc i2c address
#define HALL_INT 2 // hall effect sensor pin for interrupts
#define RTC_SELECT 4 // cc for sd card (not used)
#define MACHINE_CTRL 7 // machine control pin (high: machine runs)
#define OVERHEAT_LED 5 // overheat indicator pin
#define OVERHEAT_REST_INTERVAL 300000 // 5mins
#define WRITE_ERR_LED 6 // write error indicator pin
#define THERM_PIN A1 // thermistor pin
#define CURR_PIN A3 // current sensor pin
#define TEMP_LIMIT 100 // max motor temp
#define INTERVAL 4000 // INTERVAL for measurements
#define CURR_CALIBRATION 508 // value for centering adc value
#define CYCLE_MAX 55000 // number of cycles to run

struct time_buf {
  uint8_t secs;
  uint8_t mins;
  uint8_t hrs;
  uint8_t day;
  uint8_t mnth;
};

void record_to_sd(uint8_t temp, uint16_t rpm, uint8_t current); // record time, current, rpm and temperature
void read_rtc(time_buf* buf); // read time
uint8_t bcd_to_dec(uint8_t a); // convert bcd to dec
uint8_t read_temp(uint8_t adc_ch); // read temperature from thermistor on adc channel
uint8_t read_current(uint8_t adc_ch); // read current from sensor on adc channel
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
uint8_t current_buffer = 0; // current buffer
uint16_t rpm_buffer = 0; // rpm buffer
uint16_t cycle_iter = 0; // number of cycles passed
uint32_t rpm_iter = 0; // rpm iterator
uint32_t prev_millis, curr_millis; // timing helpers

void setup() {
  Serial.begin(9600); // open serial port
  Wire.begin(); // start i2c bus

  SD.begin(RTC_SELECT); // initialize sd card
 
  attachInterrupt(digitalPinToInterrupt(HALL_INT), hall_interrupt, FALLING);

  pinMode(MACHINE_CTRL, OUTPUT);
  pinMode(OVERHEAT_LED, OUTPUT);
  pinMode(WRITE_ERR_LED, OUTPUT);

  digitalWrite(MACHINE_CTRL, 0);
  digitalWrite(OVERHEAT_LED, 0);
  digitalWrite(WRITE_ERR_LED, 0);
}

void loop() {
  // check time
  curr_millis = millis();

  if(curr_millis - prev_millis >= INTERVAL) { // if INTERVAL reached (every 4s)
    // record time
    prev_millis = curr_millis;

    if(cycle_iter < CYCLE_MAX) { // if havent finished
      // measure temp
      temp_buffer = read_temp(THERM_PIN);
      
      // if overheating, rest
      if(temp_buffer >= TEMP_LIMIT) {
        record_to_sd(temp_buffer, 0, 0);
        digitalWrite(OVERHEAT_LED, 1); // turn on indicator led
        digitalWrite(MACHINE_CTRL, 0); // turn off machine
        delay(OVERHEAT_REST_INTERVAL);
        record_to_sd(temp_buffer, 0, 0);
        interval_iter = 0;
      }
      
      // control machine
      ++interval_iter <= 14 ? 
        digitalWrite(MACHINE_CTRL, 1) : // every 56s
        digitalWrite(MACHINE_CTRL, 0); // every 4s

      if(interval_iter == 1 || interval_iter == 8) {
        read_rpm(0); // start rpm measurement
      }
  
      // record info on 7th (28s) and 14th interval (56s)
      if(interval_iter == 7 || interval_iter == 14) {
        rpm_buffer = read_rpm(1); // record rpm
        current_buffer = read_current(CURR_PIN); // record current
        record_to_sd(temp_buffer, rpm_buffer, current_buffer);
      }
  
      if(interval_iter == 15) { // every 60s
        interval_iter = 0; // reset iteration
        cycle_iter++; // iterate cycles
      }
    }
  }

//  // SANITY CHECK
//  read_rpm(0);
//  delay(5000);
//  Serial.print(read_temp(THERM_PIN));
//  Serial.print("\t");
//  Serial.print(read_current(CURR_PIN));
//  Serial.print("\t");
//  Serial.println(read_rpm(1));
//  delay(500);
}

void record_to_sd(uint8_t temp, uint16_t rpm, uint8_t current) {
  time_buf time_buffer;

  read_rtc(&time_buffer);
  
  File data_file = SD.open("datalog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (data_file) {
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
  } else {
    digitalWrite(WRITE_ERR_LED, 1); // turn on indicator led
  }
}

uint8_t bcd_to_dec(uint8_t a) {
  return(a - 6 * (a >> 4));
}

void read_rtc(time_buf* buf) {
  // TODO: works
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
  uint16_t rpm; // rpm result
  
  if(!cntrl) {
    local_curr_millis = millis();
    
    attachInterrupt(digitalPinToInterrupt(HALL_INT), hall_interrupt, FALLING); // reattach interrupt    
    
    return(1);
  } else {
    detachInterrupt(digitalPinToInterrupt(HALL_INT)); // detach interrupt (atomic op)

    // record time
    local_prev_millis = local_curr_millis;
    local_curr_millis = millis();
    
    rpm = rpm_iter * 60000 / (local_curr_millis - local_prev_millis); // calc rpm
    
    rpm_iter = 0; // reset iterator
  
    return(rpm);
  }
}

uint8_t read_current(uint8_t adc_ch) {
  int16_t adc_val;
  
  adc_val = analogRead(adc_ch); // read adc
  adc_val = abs(adc_val - CURR_CALIBRATION); // center value
  
  // return current (sensitivity: 100mV / 1A; 2.5V@0A (centered))
  return(adc_val / 2);
}

void hall_interrupt(void) {
  rpm_iter++; // increment iterator
}

