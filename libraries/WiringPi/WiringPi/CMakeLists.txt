cmake_minimum_required(VERSION "3.19.2")

project(libwiringPi VERSION 1.0.0)

add_library(libwiringPi STATIC
	"wiringPi.c"
	"wiringSerial.c"
	"wiringShift.c"
	"piHiPri.c"
	"piThread.c"
	"wiringPiSPI.c"
	"wiringPiI2C.c"
	"softPwm.c"
	"softTone.c"
	"mcp23008.c"
	"mcp23016.c"
	"mcp23017.c"
	"mcp23s08.c"
	"mcp23s17.c"
	"sr595.c"
	"pcf8574.c"
	"pcf8591.c"
	"mcp3002.c"
	"mcp3004.c"
	"mcp4802.c"
	"mcp3422.c"
	"max31855.c"
	"max5322.c"
	"ads1115.c"
	"sn3218.c"
	"bmp180.c"
	"htu21d.c"
	"ds18b20.c"
	"rht03.c"
	"drcSerial.c"
	"drcNet.c"
	"pseudoPins.c"
	"wpiExtensions.c"
	"wiringPiLegacy.c"
)

target_include_directories(libwiringPi
	PUBLIC /
)
