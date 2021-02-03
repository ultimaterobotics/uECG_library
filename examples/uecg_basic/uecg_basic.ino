/*
Obtaining basic data from uECG device using nRF24 radio module

Important! You need RF24 library installed!
*/

#include <uECG.h>

int rf_cen = 9; //nRF24 chip enable pin
int rf_cs = 10; //nRF24 CS pin

void setup() {
  Serial.begin(115200);
  uECG.begin(rf_cs, rf_cen);
}

long last_print = 0; //we want to print values once in a while
//but we can't use delay() function - so need to remember when
//values were printed last time

void loop() 
{
  uECG.run(); //need to call this really often, therefore
  //no delays can be used in the code

  long ms = millis();
  if(ms - last_print > 500) //once per 500 milliseconds
  {
    last_print = ms; //remember last printing time
    Serial.print("BPM: ");
    Serial.println(uECG.getBPM());
    Serial.print("Battery (mV): ");
    Serial.println(uECG.getBattery());
    Serial.print("Steps: ");
    Serial.println(uECG.getSteps());
    Serial.print("RR: ");
    Serial.println(uECG.getLastRR());
  }
}

/*
Full list of data that can be received:
uint32_t getID() - returns device ID
int getBPM() - returns BPM calculated on the device
int getBattery() - returns device battery voltage in mV
int getGSR() - returns skin resistance (in arbitrary units, not calibrated)
int getSteps() - returns step count calculated on the device

float getTemperature() - returns temperature (in C) of the device,
it usually is lower than body core temperature, unless worn for a long
time under clothing. Device itself generates almost zero heat.

void getAccel(float *ax, float *ay, float *az) - get current acceleration
(uses device on-board accelerometer, but refresh rate is low)

int getHRV() - returns single number representing level of beat-to-beat 
variations, scale 0...1000

int getLastRR() - returns last RR interval in milliseconds
int getLastRRid() - returns id of the last interval (increased from 
0 to 15, then back to 0), useful for HRV calculations

uint32_t getDataCount() - returns current number of received ECG samples
but only last 8 readings are stored in memory, so read them often!

void getHRVbins(int *bins, int max_count) - fills array of HRV bins,
bin 0 has relative count of beats with variation <1%, bin 2 - with 
variation from 1% to 2% etc, max 16 bins (last bin represents also all
outliers, so can be much higher than others, especially if skin contact
is poor)

int getECG(int16_t *ecg_data, int max_count) - fills ecg_data with latest
ECG samples, not more than max_count samples, returns how many 
points were filled (maximum 8 points)
 */
