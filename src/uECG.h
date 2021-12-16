/*
  uECG.h

  Copyright (c) 2021, Ultimate Robotics

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef UECG_h
#define UECG_h

#include <RF24.h>
//#include <RF24_config.h>
//#include <nRF24L01.h>

typedef struct uECG_data
{
	uint32_t id;
	uint32_t prev_ecg_update_ms;
	uint32_t lf_data_count;
	float lf_ecg_value;
	int batt_mv;
	int BPM;
	int GSR;
	int HRV;
	int steps;
	int lastRR;
	int lastRRid;
	float temperature;
	float ax;
	float ay;
	float az;
	uint8_t hrv_bins[16];
	int16_t hf_ecg[16];
	int16_t lf_ecg[8];
};

class uECG_
{
private:
	RF24 *rf;
	uECG_data data;
	int protocol_detected;
	int need_dewhite;
	float avg_b0;
	float avg_b1;
	float avg_diff;
	uint8_t swapbits(uint8_t a);
	float decode_acc(float acc);
public:
	uECG_(void);
	void begin(int pin_cs, int pin_ce);
	void run();
	uint32_t getID();
	uint32_t getDataCount();
	int getBPM();
	int getECG(int16_t *ecg_data, int max_count);
	int getBattery();
	int getGSR();
	int getHRV();
	int getSteps();
	float getTemperature();
	void getAccel(float *ax, float *ay, float *az);
	int getLastRR();
	int getLastRRid();
	void getHRVbins(int *bins, int max_count);
};
extern uECG_ uECG;

#endif
