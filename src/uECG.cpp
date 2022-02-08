/*
  uECG.cpp

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
#include <SPI.h>
#include <RF24.h>
#include "uECG.h"

enum param_sends
{
  param_batt_bpm = 0,
  param_sdnn,
  param_skin_res,
  param_lastRR,
  param_imu_acc,
  param_imu_steps,
  param_pnn_bins,
  param_end,
  param_emg_spectrum
};

const PROGMEM uint8_t dewhite_table[31] = {2,77,61,195,248,236,82,250,161,111,57,89,131,107,163,34,4,154,123,135,241,216,165,245,66,222,114,179,6,215,70};

uECG_::uECG_(void)
{
	protocol_detected = -1;
	avg_b0 = 20;
	avg_b1 = 20;
	avg_diff = 1;
	need_dewhite = -1;
	data.prev_ecg_update_ms = millis();
	data.lf_ecg_value = 0;
	data.lf_data_count = 0;
}

uint8_t uECG_::swapbits(uint8_t a)
{ //uECG pipe address uses swapped bits order
  // reverse the bit order in a single byte
    uint8_t v = 0;
    if(a & 0x80) v |= 0x01;
    if(a & 0x40) v |= 0x02;
    if(a & 0x20) v |= 0x04;
    if(a & 0x10) v |= 0x08;
    if(a & 0x08) v |= 0x10;
    if(a & 0x04) v |= 0x20;
    if(a & 0x02) v |= 0x40;
    if(a & 0x01) v |= 0x80;
    return v;
}
float uECG_::decode_acc(float acc)
{
	float vv = acc - 128.0;
	int vm = vv;
	if(vm < 0) vm = -vm;
	float res = 0;
	if(vm > 100) //for values less than -12 and more than 12 m/s^2 precision is 0.5
	{
		res = 12.0 + (vm - 100) / 2.0;
	}
	else if(vm > 50) //for values from -12...-2 and 2...12 m/s^2 precision is 0.2
	{
		res = 2.0 + (vm - 50) / 5.0;
	}
	else
		res = vm / 25.0; //for values from -2 to 2 m/s^2 precision is 0.04
	if(vv < 0) res = -res;
	return res;
}

void uECG_::begin(int pin_cs, int pin_ce)
{
//	SPI.begin();
//	SPI.setBitOrder(MSBFIRST);
//	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

	rf = new RF24(pin_ce, pin_cs);
	uint8_t pipe_rx[8] = {0x0E, 0xE6, 0x0D, 0xA7, 0, 0, 0, 0};
	for(int x = 0; x < 8; x++) //nRF24 and uECG have different bit order for pipe address
		pipe_rx[x] = swapbits(pipe_rx[x]);

	rf->begin();
	rf->setDataRate(RF24_250KBPS);
	rf->setAddressWidth(4);
	rf->setChannel(21);
	rf->setRetries(0, 0);
	rf->setAutoAck(0);
	rf->disableDynamicPayloads();
	rf->setPayloadSize(32);
	rf->openReadingPipe(0, pipe_rx);
	rf->setCRCLength(RF24_CRC_DISABLED);
	rf->disableCRC();
	rf->startListening(); //listen for uECG data
	data.prev_ecg_update_ms = millis();
	Serial.println("uECG init");
}
void uECG_::run()
{
	if(!rf->available()) return;
	uint8_t rf_pack[33];
	rf->read(rf_pack, 32); //processing packet
	uint8_t *in_pack = rf_pack+1; //ignore 1st byte
	if(need_dewhite == 1)
		for(int x = 0; x < 31; x++)
			in_pack[x] = in_pack[x]^pgm_read_byte(dewhite_table+x);
/*	for(int x = 0; x < 31; x++)
	{
		Serial.print(in_pack[x]);
		Serial.print(' ');
	}
	Serial.println();*/
	if(need_dewhite < 0)
	{
		static uint8_t prev_pack0 = 0;
		static int dw_cnt = 0;
		float dp = 0;
		if(in_pack[0] > prev_pack0) dp = in_pack[0] - prev_pack0;
		else dp = prev_pack0 - in_pack[0];
		avg_diff *= 0.98;
		avg_diff += 0.02*dp;
		dw_cnt++;
		if(dw_cnt > 500)
		{
			if(avg_diff < 5) need_dewhite = 0;
			else need_dewhite = 1;
		}
	}
	if(protocol_detected < 1 && need_dewhite >= 0)
	{
		avg_b0 *= 0.98;
		avg_b0 += 0.02*in_pack[0];
		avg_b1 *= 0.98;
		avg_b1 += 0.02*in_pack[1];

		if(avg_b0 < 40 && avg_b1 > 50 && avg_b1 < 80) protocol_detected = 1;
		if(avg_b1 < 40 && avg_b0 > 50 && avg_b0 < 80) protocol_detected = 2;
		return;
	}
	byte pack_id = in_pack[0];
	byte message_length = in_pack[1];
	if(protocol_detected == 1)
	{
		pack_id = in_pack[1];
		message_length = in_pack[0];
	}
	if(message_length >= 32)
	{
//		Serial.print("msg too long ");
//		Serial.println(message_length);
		return;
	}
	byte chk = in_pack[message_length-1];
	byte checksum = 0;
	byte checksumP = 0;
	byte check_ok = 0;
	for(int x = 0; x < message_length-1; x++)
		checksum += in_pack[x];
	if(checksum != chk)
	{
		checksum -= in_pack[message_length-2];
		for(int x = 0; x < message_length-2; x+=2)
			checksumP += in_pack[x];
		if(checksum == in_pack[message_length-2] && checksumP == chk)
		{
			check_ok = 1;
			protocol_detected = 3;
		}
	}
	else check_ok = 1;

	if(!check_ok)
	{
		for(int x = 0; x < message_length+1; x++)
		{
			Serial.print(in_pack[x]);
			Serial.print(' ');
		}
		Serial.print("check err: ");
		Serial.print(chk);
		Serial.print(' ');
		Serial.println(checksum);
		return;
	}
	byte u1 = in_pack[2];//32-bit unit ID, unique for every uECG device
	byte u2 = in_pack[3];
	byte u3 = in_pack[4];
	byte u4 = in_pack[5];

	int ppos = 6;
	byte data_points = in_pack[ppos++];
	byte data_id = 0;
	if(protocol_detected == 3)
	{
		data_id = data_points;
		data_points = 9;
	}
	if(data_points > 10) return; //something is wrong, impossible to fit into 32-bytes packet
	data.id = (u1<<24) | (u2<<16) | (u3<<8) | u4;

	byte param_id = in_pack[ppos++];
	byte pb1 = in_pack[ppos++];
	byte pb2 = in_pack[ppos++];
	byte pb3 = in_pack[ppos++];
	//    char tmp[256];
	//    int ln = sprintf(tmp, "%d %d %d %d\n", in_pack[5], in_pack[6], in_pack[7], in_pack[8]);
	//    Serial.write(tmp, ln);
	if(param_id == param_batt_bpm)
	{
		data.batt_mv = 2000 + pb1*10;
		data.BPM = pb3;
	}
	if(param_id == param_sdnn)
	{
		int sdnn = ((pb1>>4)<<8) | pb2;
		int rmssd = (pb1&0x0F) | pb3;
	}
	if(param_id == param_skin_res)
	{
		data.GSR = (pb1<<8) | pb2;
		if(pb3 < 50)
			data.temperature = -20 + pb3;
		else if(pb3 > 220)
			data.temperature = 47 + (pb3-220);
		else 
			data.temperature = 30 + 0.1 * (float)(pb3 - 50);
	}
	if(param_id == param_lastRR) 
	{
		data.lastRRid = pb1;
		data.lastRR = (pb2<<8) | pb3;
	}
	if(param_id == param_pnn_bins)
	{
		int bin_id = pb1;
		if(bin_id < 15)
		{
			data.hrv_bins[bin_id] = pb2;
			data.hrv_bins[bin_id+1] = pb3;
		}
	}

	if(param_id == param_imu_steps) //step counter, overflows at 65536
	{
		data.steps = (pb1<<8) | pb2;
	}
	if(param_id == param_imu_acc) //current acceleration by x,y,z packed in a way to increase precision around 0, so angle can be extracted more or less reliably
	{
		data.ax = decode_acc(pb1);
		data.ay = decode_acc(pb2);
		data.az = decode_acc(pb3);
	}
	for(int x = 0; x < 16-data_points; x++)
		data.hf_ecg[x] = data.hf_ecg[x+data_points];

	int max_dv = 0, prev_val = 0;
	int prev_pos = ppos;
	for(int x = 0; x < data_points; x++)
	{
		int16_t val = (in_pack[ppos]<<8) | in_pack[ppos+1];
		ppos += 2;
		if(x > 0)
		{
			int dv = val - prev_val;
			if(dv < 0) dv = -dv;
			if(dv > max_dv) max_dv = dv;
		}
		prev_val = val;
	}

	if(max_dv > 10000) return; //some error still got past checksum
	ppos = prev_pos;
	for(int x = 0; x < data_points; x++)
	{
		int16_t val = (in_pack[ppos]<<8) | in_pack[ppos+1];
		ppos += 2;
		data.hf_ecg[16-data_points+x] = val;
		data.lf_ecg_value *= 0.8;
		data.lf_ecg_value += 0.2*(float)val;
	}
	uint32_t ms = millis();
	int lf_points = (ms - data.prev_ecg_update_ms)>>3;
	if(lf_points > 4 || lf_points < 0) lf_points = 4;
	if(lf_points < 1) return;
	data.prev_ecg_update_ms = ms;
	for(int x = 0; x < 8-lf_points; x++)
		data.lf_ecg[x] = data.lf_ecg[x+lf_points];
	for(int x = 0; x < lf_points; x++)
		data.lf_ecg[8-lf_points+x] = data.lf_ecg_value;
	data.lf_data_count += lf_points;
}
uint32_t uECG_::getID()
{
	return data.id;
}
uint32_t uECG_::getDataCount()
{
	return data.lf_data_count;
}
int uECG_::getBPM()
{
	return data.BPM;
}
int uECG_::getECG(int16_t *ecg_data, int max_count)
{
	int ret_cnt = max_count;
	if(ret_cnt > 8) ret_cnt = 8;
	for(int x = 0; x < ret_cnt; x++)
		ecg_data[x] = data.lf_ecg[8-ret_cnt+x];
	return ret_cnt;
}
int uECG_::getBattery()
{
	return data.batt_mv;
}
int uECG_::getGSR()
{
	return data.GSR;
}
int uECG_::getHRV()
{
	float sum_short = data.hrv_bins[0] + data.hrv_bins[1];
	float sum_long = data.hrv_bins[2] + data.hrv_bins[3] + data.hrv_bins[4] + data.hrv_bins[5];
	float score = sum_long / (sum_short + sum_long + 1);
	return 1000*score;
}
int uECG_::getSteps()
{
	return data.steps;
}
float uECG_::getTemperature()
{
	return data.temperature;
}
void uECG_::getAccel(float *ax, float *ay, float *az)
{
	*ax = data.ax;
	*ay = data.ay;
	*az = data.az;
}
int uECG_::getLastRR()
{
	return data.lastRR;
}
int uECG_::getLastRRid()
{
	return data.lastRRid;
}
void uECG_::getHRVbins(int *bins, int max_count)
{
	int ret_cnt = max_count;
	if(ret_cnt > 16) ret_cnt = 16;
	for(int x = 0; x < ret_cnt; x++)
		bins[x] = data.hrv_bins[x];
}

uECG_ uECG;


