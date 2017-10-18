#ifndef UM7_H
#define UM7_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

class UM7 {
public:
	
	UM7();
	
	bool encode(byte c);
	
	float roll() const { return convert_degree(_roll); }
	float pitch() const { return convert_degree(_pitch); }
	float yaw() const { return convert_degree(_yaw); }
	
	float roll_rate() const { return convert_rate(_roll_rate); }
	float pitch_rate() const { return convert_rate(_pitch_rate); }
	float yaw_rate() const { return convert_rate(_yaw_rate); }
	
	float q_a() const { return convert_quat(_qa); }
	float q_b() const { return convert_quat(_qb); }
	float q_c() const { return convert_quat(_qc); }
	float q_d() const { return convert_quat(_qd); }

private:

	short _roll, _pitch, _yaw, _roll_rate, _pitch_rate, _yaw_rate;
	short _qa, _qb, _qc, _qd;

	enum {STATE_ZERO,STATE_S,STATE_SN,STATE_SNP,STATE_PT,STATE_DATA,STATE_CHK1,STATE_CHK0};

	int state;
	
	byte packet_type;
	byte address;
	bool packet_is_batch;
	byte batch_length;
	bool packet_has_data;
	byte data[30];
	byte data_length;
	byte data_index;

	byte checksum1;		// First byte of checksum
	byte checksum0;		// Second byte of checksum

	byte read_index; // for the init_read() and next_read() functions

	unsigned short checksum10;			// Checksum received from packet
	unsigned short computed_checksum;	// Checksum computed from bytes received
	
	bool checksum(void);
	
	void save(void);
	
	void init_read();
	void next_short(short* dst);
	
	static float convert_degree(short deg);
	static float convert_rate(short rate);
	static float convert_quat(short quat);
};

#endif
