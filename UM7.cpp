#include "UM7.h"

#define DREG_QUAT_AB         0x6D	// Packet address sent from the UM7 that contains quaternion values.
#define DREG_EULER_PHI_THETA 0x70	// Packet address sent from the UM7 that contains roll,pitch,yaw and rates.

#define DD 91.02222    // divider for degrees
#define DR 16.0        // divider for rate
#define DQ 29789.09091 // divider for quaternion element

UM7::UM7() : state(STATE_ZERO){}	// Default constructor

bool UM7::encode(byte c){
	
	switch(state){
	case STATE_ZERO:
		if (c == 's'){
			state = STATE_S;		// Entering state S from state Zero
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_S:
		if (c == 'n'){
			state = STATE_SN;		// Entering state SN from state S
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_SN:
		if (c == 'p'){
			state = STATE_SNP;		// Entering state SNP from state SN.  Packet header detected.
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_SNP:
		state = STATE_PT;			// Entering state PT from state SNP.  Decode packet type.
		packet_type = c;
		packet_has_data = (packet_type >> 7) & 0x01;
		packet_is_batch = (packet_type >> 6) & 0x01;
		batch_length    = (packet_type >> 2) & 0x0F;
		if (packet_has_data){
			if (packet_is_batch){
				data_length = 4 * batch_length;	// Each data packet is 4 bytes long
			} else {
				data_length = 4;
			}
		} else {
			data_length = 0;
		}  
		return false;
	case STATE_PT:
		state = STATE_DATA;		// Next state will be READ_DATA.  Save address to memory. (eg 0x70 for a DREG_EULER_PHI_THETA packet)
		address = c;
		data_index = 0;
		return false;
	case STATE_DATA:			//  Entering state READ_DATA.  Stay in state until all data is read.
		data[data_index] = c;
		data_index++;
		if (data_index >= data_length){
			state = STATE_CHK1;	//  Data read completed.  Next state will be CHK1
		}
		return false;
	case STATE_CHK1:			// Entering state CHK1.  Next state will be CHK0
		state = STATE_CHK0;
		checksum1 = c;
		return false;
	case STATE_CHK0: 				
		state = STATE_ZERO;		// Entering state CHK0.  Next state will be state Zero.
		checksum0 = c;
		return checksum();
	}
}

bool UM7::checksum(){
	checksum10  = checksum1 << 8;	// Combine checksum1 and checksum0
	checksum10 |= checksum0;
	computed_checksum = 's' + 'n' + 'p' + packet_type + address;
	for (int i = 0; i < data_length; i++){
		computed_checksum += data[i];
	}
	if (checksum10 == computed_checksum){
		save();
		return true;
	} else {
		return false;
	}
}

void UM7::save(){
	init_read();
	
	switch(address){

	case DREG_QUAT_AB :
		if(packet_is_batch){
		  next_short(&_qa);
		  next_short(&_qb);
		  next_short(&_qc);
		  next_short(&_qd);
		}else{
		  next_short(&_qa);
		  next_short(&_qb);
		}
		break;
	case DREG_EULER_PHI_THETA :		// data[6] and data[7] are unused.
		if(packet_is_batch){
		  next_short(&_roll);
		  next_short(&_pitch);
		  next_short(&_yaw);
		  
		  short unused;
		  next_short(&unused);
		  
		  next_short(&_roll_rate);
		  next_short(&_pitch_rate);
		  next_short(&_yaw_rate);
		}else{
		  next_short(&_roll);
		  next_short(&_pitch);
		}
		break;
	}
}

void UM7::init_read() {
  read_index = 0;
}

void UM7::next_short(short* dst) {
  *dst = (data[read_index++] << 8) | data[read_index++];
}

float UM7::convert_degree(short deg) {
  return deg / DD;
}

float UM7::convert_rate(short rate) {
  return rate / DR;
}

float UM7::convert_quat(short quat) {
  return quat / DQ;
}

