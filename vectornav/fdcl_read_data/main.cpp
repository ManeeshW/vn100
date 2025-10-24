#include "fdcl_vn100.h"

fdcl_vn100 imu;

int main(int argc, char *argv[])
{
	// imu.openAscii();
	imu.openBinary();
	return 0;
}
