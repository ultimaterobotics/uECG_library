/*
Obtaining basic data from uECG device using nRF24 radio module
with plotting ECG on OLED display, and sending ECG data to Serial

Important! You need RF24 library installed!
*/

#include <uECG.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int rf_cen = 9; //nRF24 chip enable pin
int rf_cs = 10; //nRF24 CS pin

void setup() {
  Serial.begin(115200); //serial output - very useful for debugging
  while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
//  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(100);
  uECG.begin(rf_cs, rf_cen);
  delay(100);

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.display();
  delay(100);
  Serial.println("after display");
}

uint32_t prev_data_count = 0;
uint32_t prev_displ = 0;

uint8_t ecg_screen[128];
int ecg_screen_len = 128;
float ecg_avg = 0;
float ecg_max = 1;
float ecg_min = -1;
int ecg_size = 40;

int displ_phase = 0;

void loop() 
{
  uECG.run();
  uint32_t data_count = uECG.getDataCount();
  int new_data = data_count - prev_data_count;
  prev_data_count = data_count;
  if(new_data > 0)
  {
    uint32_t ms = millis();
    int16_t ecg_data[8];
    uECG.getECG(ecg_data, new_data);
    for(int x = 0; x < new_data; x++)
      Serial.println(ecg_data[x]);
      
    for(int x = new_data; x < ecg_screen_len; x++)
      ecg_screen[x-new_data] = ecg_screen[x];
    for(int x = 0; x < new_data; x++)
    {
      ecg_avg *= 0.99;
      ecg_avg += 0.01*ecg_data[x];
      ecg_max = ecg_max*0.995 + ecg_avg*0.005;
      ecg_min = ecg_min*0.995 + ecg_avg*0.005;
      if(ecg_data[x] > ecg_max) ecg_max = ecg_data[x];
      if(ecg_data[x] < ecg_min) ecg_min = ecg_data[x];
      int ecg_y = 63-ecg_size*(ecg_data[x] - ecg_min) / (ecg_max - ecg_min + 1);
      ecg_screen[ecg_screen_len-1-new_data+x] = ecg_y;
    }

    if(ms - prev_displ > 30)
    {
      prev_displ = ms;
      if(displ_phase == 0)
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("BPM: ");
        display.println(uECG.getBPM());
        display.print(" RR: ");
        display.println(uECG.getLastRR());
        display.print("steps: ");
        display.print(uECG.getSteps());
        int batt_mv = uECG.getBattery();
        int batt_perc = (batt_mv - 3300)/8;
        if(batt_perc < 0) batt_perc = 0;
        if(batt_perc > 100) batt_perc = 100;
        display.drawLine(110, 0, 127, 0, WHITE);
        display.drawLine(110, 10, 127, 10, WHITE);
        display.drawLine(110, 0, 110, 10, WHITE);
        display.drawLine(127, 0, 127, 10, WHITE);
        int bat_len = batt_perc / 6;
        for(int x = 1; x < 10; x++)
          display.drawLine(110, x, 110+bat_len, x, WHITE);
      }
      if(displ_phase == 1) 
      {
        for(int x = 1; x < ecg_screen_len/2; x++)
          display.drawLine(x-1, ecg_screen[x-1], x, ecg_screen[x], WHITE);
      }      
      if(displ_phase == 2) 
      {
        for(int x = ecg_screen_len/2; x < ecg_screen_len-1; x++)
          display.drawLine(x-1, ecg_screen[x-1], x, ecg_screen[x], WHITE);
      }
      if(displ_phase == 3) 
        display.display();
      displ_phase++;
      if(displ_phase > 3) displ_phase = 0;
    }
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
