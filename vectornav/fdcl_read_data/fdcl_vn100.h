#ifndef FDCL_VN_100_H
#define FDCL_VN_100_H

#include <iostream>

// Include this header file to get access to the EzAsyncData class.
#include "vn/ezasyncdata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// #include "vectornav.h"
// #include "fdcl_param.h"
// #include "misc_matrix_func.h"
//
class fdcl_vn100
{
public:
	fdcl_vn100();
	~fdcl_vn100();
	//
	// string port; //"/dev/ttyTHS2";
	string port;
	// const string port = "/dev/tty.usbserial-FTWV7X38";

	// int baud_rate
	uint32_t baud_rate;
	//
	// 	static void callback(void* sender, VnDeviceCompositeData* data);
	//
	// 	void load_config(fdcl_param& );
	void openAscii();
	void readAscii(string, int);

	void openBinary();
	void readBinary(string, int);

	// static void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

	// void open();
	// void loop();
	// 	void close();
	//
	private:
		EzAsyncData* ez;
		static void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

	// 	VN_ERROR_CODE errorCode;
	// 	Vn100 vn100;
};

#endif
