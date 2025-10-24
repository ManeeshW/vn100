#include "fdcl_vn100.h"

fdcl_vn100::~fdcl_vn100()
{
}


fdcl_vn100::fdcl_vn100()
{
}


void fdcl_vn100::openBinary()
{
	port  = "/dev/ttyUSB0";
	baud_rate = 115200;

	readBinary(port, baud_rate);
}


void fdcl_vn100::readBinary(string port, int baud_rate)
{
	// create a VnSensor object and use it to connect to sensor.
	VnSensor vs;
	cout << "Connecting to IMU at " << port << " .." << endl;
	vs.connect(port, baud_rate);

	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	cout << "Setting up binary output register .." << endl;
	vs.writeAsyncDataOutputType(VNOFF);  // disable ASCII output

	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		200, // 4 Hz
		COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL, // Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE
	);
	vs.writeBinaryOutput1(bor);

	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);
	cout << "Starting sleep..." << endl;
	Thread::sleepSec(5);
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();

}


void fdcl_vn100::asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
    if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
    {
        vec3f ypr;
        p.parseVNYPR(&ypr);
        cout << "ASCII Async YPR: " << ypr << endl;
        return;
    }
    if (p.type() == Packet::TYPE_BINARY)
    {
        // First make sure we have a binary packet type we expect since there
        // are many types of binary output types that can be configured.
        if (!p.isCompatible(
            COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_NONE,
            INSGROUP_NONE))
            // Not the type of binary packet we are expecting.
            return;
        // Ok, we have our expected binary output packet. Since there are many
        // ways to configure the binary data output, the burden is on the user
        // to correctly parse the binary packet. However, we can make use of
        // the parsing convenience methods provided by the Packet structure.
        // When using these convenience methods, you have to extract them in
        // the order they are organized in the binary packet per the User Manual.
        vec3f ypr = p.extractVec3f();
				vec3f ang_rate = p.extractVec3f();
				vec3f accel = p.extractVec3f();
        cout << "Binary Async YPR: " << ypr;
        cout << "\tBinary Async W: " << ang_rate;
				cout << "\tBinary Async a: " << accel << endl;
    }
}


void fdcl_vn100::openAscii()
{
	port  = "/dev/ttyUSB0";
	baud_rate = 115200;

	readAscii(port, baud_rate);
}


void fdcl_vn100::readAscii(string port, const int baud_rate)
{

	// We create and connect to a sensor by the call below.
	ez = EzAsyncData::connect(port, baud_rate);

	// Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
	for (int i = 0; i < 5; i++)
	{
		Thread::sleepMs(200);

		// This reads the latest data that has been processed by the EzAsyncData class.
		CompositeData cd = ez->currentData();

		// Make sure that we have some yaw, pitch, roll data.
		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// Most of the asynchronous data handling is done by EzAsyncData but there are times
	// when we wish to configure the sensor directly while still having EzAsyncData do
	// most of the grunt work.This is easily accomplished and we show changing the ASCII
	// asynchronous data output type here.
	try
	{
		ez->sensor()->writeAsyncDataOutputType(VNYPR);
	}
	catch (...)
	{
		cout << "Error setting async data output type." << endl;
	}

	cout << "[New ASCII Async Output]" << endl;

	// We can now display yaw, pitch, roll data from the new ASCII asynchronous data type.
	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(200);

		CompositeData cd = ez->currentData();

		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// The CompositeData structure contains some helper methods for getting data
	// into various formats. For example, although the sensor is configured to
	// output yaw, pitch, roll, our application might need it as a quaternion
	// value. However, if we query the quaternion field, we see that we don't
	// have any data.

	cout << "HasQuaternion: " << ez->currentData().hasQuaternion() << endl;

	// Uncommenting the line below will cause an exception to be thrown since
	// quaternion data is not available.

	// cout << "Current Quaternion: " << ez->currentData().quaternion() << endl;

	// However, the CompositeData structure provides the anyAttitude field
	// which will perform the necessary conversions automatically.

	cout << "[Quaternion from AnyAttitude]" << endl;

	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(200);

		// This reads the latest data that has been processed by the EzAsyncData class.
		CompositeData cd = ez->currentData();

		// Make sure that we have some attitude data.
		if (!cd.hasAnyAttitude())
		cout << "Attitude Unavailable." << endl;
		else
		cout << "Current Quaternion: " << cd.anyAttitude().quat() << endl;
	}

	// Throughout this example, we have been using the ez->currentData() to get the most
	// up-to-date readings from the sensor that have been processed. When called, this
	// method returns immediately with the current values, thus the reason we have to
	// put the Thread::sleepMs(200) in the for loop. Otherwise, we would blaze through
	// the for loop and just print out the same values. The for loop below illustrates
	// this.

	cout << "[For Loop Without Sleep]" << endl;

	for (int i = 0; i < 25; i++)
	{
		CompositeData cd = ez->currentData();

		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// Often, we would like to get and process each packet received from the sensor.
	// This is not realistic with ez->currentData() since it is non-blocking and we
	// would also have to compare each CompositeData struture for changes in the data.
	// However, EzAsyncData also provides the getNextData() method which blocks until
	// a new data packet is available. The for loop below shows how to output each
	// data packet received from the sensor using getNextData().

	cout << "[getNextData Method]" << endl;

	for (int i = 0; i < 25; i++)
	{
		CompositeData cd = ez->getNextData();

		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	ez->disconnect();

	delete ez;

}
