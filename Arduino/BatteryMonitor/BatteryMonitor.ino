#include <OneWire.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <ina219.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <math.h>
#include <SPI.h>
#include <rf24HQ.h>


#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// Pins used
// Digital
// 0 - Serial RX
// 1 - Serial TX
// 2
// 3
// 4
// 5
// 6 - Charger PWM pin
// 7
// 8 - nRF enable
// 9 - nRF CSN
// 10 - SD CS
// 11 - MOSI
// 12 - MISO
// 13 - SCK
//
// Analog
// 0 
// 1
// 2
// 3
// 4 - SDA (for RTC and INA219)
// 5 - SDC


// Real Time Clock
RTC_DS1307 RTC;

// SD shield chip select pin
const int chipSelect = 10; 

// Battery monitor
INA219 monitor;

// the logging file
File logfile;


#define SYNC_INTERVAL 10 // seconds between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define UNKNOWN 0
#define ATREST 1
#define CHARGING 2
#define LOAD 3
#define BULK 4
#define ABSORB 5
#define FLOAT 6

#define V_ABSORB 14.4

// Global variables for measurements
DateTime now = 0;
float voltage = 0;
float current = 0;
float temp = 0;
int soc = 0;
int capacity = 345;
int state = UNKNOWN;
int laststate = UNKNOWN;
long laststatechange = 0;
float ahcharge = 0;
float ahload = 0;
long lastreading = 0;
int output_level = 0;

float peukert_c = 1.3;
float charge_c = 1.2;

volatile int f_wdt=1;

rf24 rf(8,9,100,12);

DeviceAddress T1 = { 0x28, 0x95, 0x09, 0xEA, 0x03, 0x00, 0x00, 0x01 };
OneWire oneWire(A0);
DallasTemperature sensors(&oneWire);

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
}

void setup () {
    Serial.begin(9600);
    Wire.begin();
    RTC.begin();

    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    
    monitor.begin(64);
    // monitor.configure(0, 3, 3, 3, 7);
    monitor.configure(0, 3, 12, 12, 7);

  
    // calibration of equations and device
    // shunt_val 		= value of shunt in Ohms
    // v_shunt_max 		= maximum value of voltage across shunt
    // v_bus_max 		= maximum voltage of bus
    // i_max_expected 	= maximum current draw of bus + shunt
    // default values are for a 0.25 Ohm shunt on a 5V bus with max current of 1A
    monitor.calibrate(0.0015, 0.075, 15, 50);
    
    // see if the card is present and can be initialized:
    if (SD.begin(chipSelect)) {
       Serial.println("card initialized.");
    } else {
       error("Card failed, or not present");
    }
    
    // create a new file
//    logfile = SD.open("LOG2.csv", FILE_WRITE);
//    if (!logfile) {
//      Serial.println("Error creating file");
//    }
    
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE);
        break;  // leave the loop!
      }
    }
  
    if (logfile) {
      Serial.print("Logging to: ");
      Serial.println(filename);
    } else {
      error("couldnt create file");
    }
    
    logfile.println("millis,voltage,current");
    
    // Set up Mirf
    Serial.println("Starting Mirf");
    rf.begin(250000, NULL);
    rf.setRxAddr(1, "BM001");
    rf.setTxAddr("DISP1");

    Serial.println("Mirf running");

    /*** Setup the WDT ***/
  
    /* Clear the reset flag. */
    MCUSR &= ~(1<<WDRF);
  
    /* In order to change WDE or the prescaler, we need to
     * set WDCE (This will allow updates for 4 clock cycles).
     */
    WDTCSR |= (1<<WDCE) | (1<<WDE);

    /* set new watchdog timeout prescaler value */
    WDTCSR = 1<<WDP1 | 1<<WDP2; /* 1.0 seconds */

    /* Enable the WD interrupt (note no reset). */
    WDTCSR |= _BV(WDIE);

    Serial.println("Initialisation complete.");

}

void logtocard () {
  long nowut = now.unixtime();
  logfile.print(nowut);
  logfile.print(",");
  logfile.print(current, 2);
  logfile.print(",");
  logfile.print(voltage, 2);
  logfile.print(",");
  logfile.print(temp, 2);
  logfile.print(",");
  logfile.print(soc);
  logfile.print(",");
  logfile.print(getsoc());
  logfile.println("");
  
  if ((nowut - syncTime) > SYNC_INTERVAL) {
    syncTime = nowut;
    logfile.flush();
    Serial.println("Flushing to SD card");
  }
}

void updatesensors () {
   now = RTC.now();   
   current = monitor.shuntCurrent() * 2.3; /* no idea why multiply by 2.3 */
   voltage = (monitor.busVoltage() + monitor.shuntVoltage());

   sensors.requestTemperaturesByAddress(T1);
   temp = sensors.getTempC(T1);
}

void outputdebug () {
  Serial.print(now.unixtime());
  Serial.print(",");
  Serial.print(current, 2);
  Serial.print(",");
  Serial.print(voltage, 2);
  Serial.print(",");
  Serial.print(temp, 2);
  Serial.print(",");
  Serial.print(soc);
  Serial.print(",");
  Serial.print(getsoc());
  Serial.println("");
  Serial.flush();
  while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
     UCSR0A |= 1 << TXC0;  // mark transmission not complete
  while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete

}

void calculatestate () {
  if ( -0.1 < current && current < 0.1) {
    state = ATREST;
  } else if (current < 0) {
    state = LOAD;
  } else if (current > 0) {
    state = CHARGING;
  }
  
  if (state != laststate) {
    laststatechange = millis();
    laststate = state;
  }  
}

void calculatesoc() {
  if (state == ATREST) {
    if ((laststate == LOAD && laststatechange + 300000 < millis()) || 
         laststate == CHARGING && laststatechange + 3600000 < millis() ) {
      int estsoc = getsoc();
      soc = 100 - ((12.80 - voltage) * 100);
      capacity = (int) (capacity * soc / estsoc);
      Serial.println("estsoc:");
      Serial.println(estsoc);
      Serial.println("real soc:");
      Serial.println(soc);
      Serial.println("ahload:");
      Serial.println(ahload);
      Serial.println("ahcharge:");
      Serial.println(ahcharge);
      Serial.println("new capacity:");
      Serial.println(capacity);
      Serial.println("");
      ahload = 0;
      ahcharge = 0;
      
    }
  }
}

int getsoc() {
  // return an estimated SoC based on last real reading and coloumb count
  return (int)soc - ((ahload - ahcharge) / capacity * 100);
}

void countcoulombs() {
   if (lastreading == 0 || current == 0) {
     return;
   }
   int interval = millis() - lastreading;
   
   if (current > 0) { // load
     ahload += (pow(current, peukert_c)  * interval / 3600000); // milliseconds in an hour
   } else { // charge
     ahcharge += (current * charge_c * interval / 3600000); // milliseconds in an hour
   }
   
   lastreading = millis();
}

void transmitdata() {
  // We use LLAP aa a protocol, with the following messages
  // aABT120.5--- Temp T1 20.5 degrees
  // aABT218.5--- Temp T2 18.5 degrees
  // aABV112.48-- Voltage V1 12.48V
  // aABI1-50.4-- Current I1 -50.4A
  // aABSOC100--- State of Charge 100%
  // aABCBULK---- Charger Bulk stage
  // aABCFLOAT--- Charger Float stage
  // aABCABSORP-- Charger Absorption stage
  // aABCEQ------ Charger Equilisation stage  
  // aABCREST---- Charger Rest  

    rf.enableTx();
  
    char msg[13];
   
    strcpy(msg, "aABV1-------");    
    dtostrf(voltage, 0, 2, &msg[5]);    
    msg[strlen(msg)] = '-';
    rf.send((byte *)&msg, 12);
  
    strcpy(msg, "aABI1-------");
    dtostrf(current, 0, 2, &msg[5]);    
    msg[strlen(msg)] = '-';
    rf.send((byte *)&msg, 12);
    
      
    strcpy(msg, "aABT1-------");
    dtostrf(temp, 0, 2, &msg[5]);    
    msg[strlen(msg)] = '-';
    rf.send((byte *)&msg, 12);

//    switch(state) {
//      case ATREST:
//        strcpy(msg, "aABCREST----");
//        rf.send((byte *)&msg, 12);
//        break;
//      case LOAD:
//        strcpy(msg, "aABCREST----");
//        rf.send((byte *)&msg, 12);
//        break;
//      case CHARGING:
//        strcpy(msg, "aABCREST----");
//        rf.send((byte *)&msg, 12);
//        break;
//    }
    
    rf.powerDown();

}


// watchdog interrupt
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
  }
  else
  {
    Serial.println("WDT Overrun!!!");
  }
}

void enterSleep(void) { 
  f_wdt = 0;
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  /* Re-enable the peripherals. */
  power_all_enable();
  delay(10); /* wait for things to stabilise after we wake up */
}

void loop () {
  if(f_wdt != 1) { 
    return;
  }
  updatesensors();
//  calculatestate();
//  calculatesoc();
//  countcoulombs();
  logtocard();
  outputdebug();
  enterSleep();
  transmitdata();  
  
}
