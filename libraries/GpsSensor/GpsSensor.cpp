#include "GpsSensor.h"
// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

	// If you're using the Adafruit GPS shield, change 
	// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
	// and make sure the switch is set to SoftSerial

	// If using software serial, keep this line enabled
	// (you can change the pin numbers to match your wiring):
	SoftwareSerial mySerial = SoftwareSerial(3, 2);

	// If using hardware serial (e.g. Arduino Mega), comment out the
	// above SoftwareSerial line, and enable this line instead
	// (you can change the Serial number to match your wiring):

	//HardwareSerial mySerial = Serial1;


	Adafruit_GPS GPS = Adafruit_GPS(&mySerial);

	bool usingInterrupt;
GpsSensor::GpsSensor()
{
}
void GpsSensor::Init()
{
	

	// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
	// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
	#define GPSECHO  false

	// this keeps track of whether we're using the interrupt
	// off by default!
	usingInterrupt = false;
	//void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


	Serial.begin(115200);

	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS.begin(9600);

	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time

	// Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);

	// the nice thing about this code is you can have a timer0 interrupt go off
	// every 1 millisecond, and read data from the GPS for you. that makes the
	// loop code a heck of a lot easier!
	useInterrupt(true);

	delay(1000);
	// Ask for firmware version
	mySerial.println(PMTK_Q_RELEASE);
}
bool GpsSensor::GetData(float array[])
{
	// in case you are not using the interrupt above, you'll
	// need to 'hand query' the GPS, not suggested :(
	if (!usingInterrupt) {
		// read data from the GPS in the 'main loop'
		char c = GPS.read();
		// if you want to debug, this is a good time to do it!
		/*if (GPSECHO)
			if (c) Serial.print(c);*/
	}

	// if a sentence is received, we can check the checksum, parse it...
	if (GPS.newNMEAreceived()) {
		// a tricky thing here is if we print the NMEA sentence, or data
		// we end up not listening and catching other sentences! 
		// so be very wary if using OUTPUT_ALLDATA and trytng to print out data
		//Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

		if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
			return 0;  // we can fail to parse a sentence in which case we should just wait for another
	}

  array[0] = millis();
  array[1] = GPS.fix;
  array[2] = GPS.fixquality;
  if (GPS.fix)
  {
	  array[3] = GPS.latitude;
	  array[4] = GPS.longitude;
	  array[5] = GPS.satellites;
	  array[6] = GPS.altitude;
	  array[7] = GPS.angle;
	  array[8] = GPS.speed;
  }
  else
  {
	  //if no fix, then set the values to 0
	  array[3] = 0;
	  array[4] = 0;
	  array[5] = 0;
	  array[6] = 0;
	  array[7] = 0;
	  array[8] = 0;
  }

  return 1;
}
bool GpsSensor::SendData(float pressureToSet)
{
  return 0;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void GpsSensor::useInterrupt(bool v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}