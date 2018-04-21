#include <ESP8266WiFi.h>	//https://github.com/esp8266/Arduino
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>	//https://github.com/tzapu/WiFiManager
#include <NeoPixelBus.h>
#include <ArduinoOTA.h>


#define HOSTNAME "ESP8266-OTA-"

#define NB_7SEGMENTS 6
#define NB_DOT       2
#define DEFAULT_COLOR cyan

uint16_t colorSaturation = 16 ; 
uint16_t PixelCount = NB_7SEGMENTS * 7 + 2 * NB_DOT;

NeoPixelBus < NeoGrbFeature, Neo800KbpsMethod > strip (PixelCount);


unsigned int localPort = 2390;	// local port to listen for UDP packets
IPAddress timeServerIP;		// time.nist.gov NTP server address
//const char* ntpServerName = "time.nist.gov";
const char *ntpServerName = "192.168.1.205";
const int NTP_PACKET_SIZE = 48;	// NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];	//buffer to hold incoming and outgoing packets
unsigned char HzLoop = 0;
unsigned long epoch = 0;

unsigned long previousMillis = 0;	// will store last time LED was updated
const long interval = 1000;	// interval at which to blink (milliseconds)



// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

unsigned long value = 0;
uint16_t SegmentStart[] = {
  0,
  7,
  16,
  23,
  32,
  39
};

uint16_t DotStart[] = {
  14,
  30
};

uint16_t SegmentPattern[] = {
  0b01111110,			/* 0 */
  0b00011000,			/* 1 */
  0b01101101,			/* 2 */
  0b00111101,			/* 3 */
  0b00011011,			/* 4 */
  0b00110111,			/* 5 */
  0b01110111,			/* 6 */
  0b00011100,			/* 7 */
  0b01111111,			/* 8 */
  0b00111111,			/* 9 */
  0b00000000			/*   */
};

// send an NTP request to the time server at the given address
unsigned long
sendNTPpacket (IPAddress & address)
{
  //Serial.println ("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset (packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;	// LI, Version, Mode
  packetBuffer[1] = 0;		// Stratum, or type of clock
  packetBuffer[2] = 6;		// Polling Interval
  packetBuffer[3] = 0xEC;	// Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket (address, 123);	//NTP requests are to port 123
  udp.write (packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket ();
}

void
setup ()
{
  // put your setup code here, to run once:
  Serial.begin (115200);
  while (!Serial);		// wait for serial attach

  Serial.println ("Initializing...");
  Serial.flush ();


  WiFiManager wifiManager;
  //reset saved settings
  //wifiManager.resetSettings();
  wifiManager.autoConnect ("AutoConnectAP");

  Serial.println ("\r\n");
  Serial.print ("Chip ID: 0x");
  Serial.println (ESP.getChipId (), HEX);

  // Set Hostname.
  String hostname (HOSTNAME);
  hostname += String (ESP.getChipId (), HEX);
  WiFi.hostname (hostname);

  //if you get here you have connected to the WiFi
  Serial.println ("connected...)");

  if(WiFi.status() == WL_CONNECTED)
  {
    // ... print IP Address
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  

  Serial.println ("Starting UDP");
  udp.begin (localPort);
  Serial.print ("Local port: ");
  Serial.println (udp.localPort ());

  // this resets all the neopixels to an off state
  strip.Begin ();
  strip.Show ();

}

void
loop ()
{
  uint16_t display;
  uint16_t segment;
  uint16_t dot;
  unsigned long currentMillis = millis ();

  RgbColor red (colorSaturation, 0, 0);
  RgbColor green (0, colorSaturation, 0);
  RgbColor blue (0, 0, colorSaturation);
  RgbColor white (colorSaturation);
  RgbColor yellow (colorSaturation, colorSaturation, 0);
  RgbColor cyan (0, colorSaturation, colorSaturation);
  RgbColor black (0);
  
  if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      epoch++;
/*
      Serial.println (HzLoop);
      Serial.print ("epoch : ");
      Serial.println (epoch);
*/
      if ((HzLoop % 60) == 0)
	{
	  HzLoop = 0;
	  WiFi.hostByName (ntpServerName, timeServerIP);
	  sendNTPpacket (timeServerIP);	// send an NTP packet to a time server
	  // wait to see if a reply is available
	  delay (300);
	  int cb = udp.parsePacket ();
	  if (!cb)
	    {
	     /* Serial.println ("no packet yet");*/
	    }
	  else
	    {
	      HzLoop++;
       /*
	      Serial.print ("packet received, length=");
	      Serial.println (cb);*/
	      // We've received a packet, read the data from it
	      udp.read (packetBuffer, NTP_PACKET_SIZE);	// read the packet into the buffer

	      unsigned long highWord =
		word (packetBuffer[40], packetBuffer[41]);
	      unsigned long lowWord =
		word (packetBuffer[42], packetBuffer[43]);
	      // combine the four bytes (two words) into a long integer
	      // this is NTP time (seconds since Jan 1 1900):
	      unsigned long secsSince1900 = highWord << 16 | lowWord;
	      /* Serial.print ("Seconds since Jan 1 1900 = ");
	      Serial.println (secsSince1900);*/

	      // now convert NTP time into everyday time:
	      /*Serial.print ("Unix time = ");*/
	      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
	      const unsigned long seventyYears = 2208988800UL;
	      // subtract seventy years:
	      epoch = secsSince1900 - seventyYears;
	      epoch += 1 * 3600;

	      // print Unix time:
	      /*Serial.print ("new epoch : ");
	      Serial.println (epoch);*/
	    }
	}
      else
	{
	  HzLoop++;
	}
#if 0
      // print the hour, minute and second:
      Serial.print ("The UTC time is ");	// UTC is the time at Greenwich Meridian (GMT)
      Serial.print ((epoch % 86400L) / 3600);	// print the hour (86400 equals secs per day)
      Serial.print (':');
      if (((epoch % 3600) / 60) < 10)
	{
	  // In the first 10 minutes of each hour, we'll want a leading '0'
	  Serial.print ('0');
	}
      Serial.print ((epoch % 3600) / 60);	// print the minute (3600 equals secs per minute)
      Serial.print (':');
      if ((epoch % 60) < 10)
	{
	  // In the first 10 seconds of each minute, we'll want a leading '0'
	  Serial.print ('0');
	}
      Serial.println (epoch % 60);	// print the second
#endif
      for (display = 0; display < NB_7SEGMENTS; display++)
	{
	  switch (display)
	    {
	    case 0:
	      value = (epoch % 10);
	      break;
	    case 1:
	      value = (epoch % 60) / 10;
	      break;
	    case 2:
	      value = ((epoch % 3600) / 60) % 10;
	      break;
	    case 3:
	      value = ((epoch % 3600) / 60) / 10;
	      break;
	    case 4:
	      value = ((epoch % 86400L) / 3600) % 10;
	      break;
	    case 5:
	      value = ((epoch % 86400L) / 3600) / 10;
	      break;
	    default:
	      value = 10;
	    }
	  for (segment = 0; segment < 7; segment++)
	    {
	      strip.SetPixelColor (segment + (SegmentStart[display]),
				   (SegmentPattern[value] & (1 << segment)) ?
				   DEFAULT_COLOR : black);
	    }
	}
      for (dot = 0; dot < NB_DOT; dot++)
	{
	  strip.SetPixelColor (DotStart[dot],
			       ((epoch % 2) == 1) ? DEFAULT_COLOR : black);
	  strip.SetPixelColor (DotStart[dot] + 1,
			       ((epoch % 2) == 1) ? DEFAULT_COLOR : black);
	}
      strip.Show ();
    }

      int sensorValue = analogRead(A0);
      float voltage = sensorValue * (3.2 / 1023.0);
      if  (voltage <  0.50) {colorSaturation = 64;}
      else if ((voltage >=  0.50) && (voltage < 2.0)) {colorSaturation = 127;}
      else if (voltage >=  2.0) {colorSaturation = 255;}


}
