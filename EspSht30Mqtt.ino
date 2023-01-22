#ifdef ESP8266
#include <ESP8266WiFi.h> // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#else
#include <WiFi.h> // Arduino Wi-Fi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#endif

#include "Adafruit_SHT31.h" // Driver library for the SHT30.  This library includes Wire.h.
#include "ESPmDNS.h"			 // Library for multicast DNS, needed for Over-The-Air updates.
#include "privateInfo.h"	 // Location of Wi-Fi and MQTT settings.
#include <ArduinoJson.h>	 // ArduinoJson by Benoît Blanchon: https://arduinojson.org/
#include <ArduinoOTA.h>		 // Arduino Over-The-Air updates.
#include <PubSubClient.h>	 // MQTT client by Nick O'Leary: https://github.com/knolleary/pubsubclient
#include <WiFiUdp.h>			 // Arduino Over-The-Air updates.

#define LED 2


char ipAddress[16];									// A character array to hold the IP address.
char macAddress[18];									// A character array to hold the MAC address, and append a dash and 3 numbers.
long rssi;												// A global to hold the Received Signal Strength Indicator.
unsigned int printInterval = 15000;				// How long to wait between stat printouts.
unsigned long printCount = 0;						// A counter of how many times the stats have been published.
unsigned long lastPrintTime = 0;					// The last time a MQTT publish was performed.
unsigned long lastBrokerConnect = 0;			// The last time a MQTT broker connection was attempted.
unsigned long brokerCoolDown = 7000;			// How long to wait between MQTT broker connection attempts.
unsigned long wifiConnectionTimeout = 15000; // The amount of time to wait for a Wi-Fi connection.
const unsigned int MCU_LED = 2;					// The GPIO which the onboard LED is connected to.
//const char *hostname = "AdamsEsp32Fleet";					  // The hostname.  Defined in privateInfo.h
const char *tempCTopic = "AdamsSensors/sht30/tempC";		  // The MQTT Celsius temperature topic.
const char *tempFTopic = "AdamsSensors/sht30/tempF";		  // The MQTT Fahrenheit temperature topic.
const char *humidityTopic = "AdamsSensors/sht30/humidity"; // The MQTT humidity topic.
const char *rssiTopic = "AdamsSensors/rssi";					  // The RSSI topic.
const char *macTopic = "AdamsSensors/mac";					  // The MAC address topic.
const char *ipTopic = "AdamsSensors/ip";						  // The IP address topic.
bool enableHeater = false;											  // The status of the SHT30 heating element.
float sht30TempCArray[] = { 21.12, 21.12, 21.12 };			  // An array to hold the 3 most recent Celsius values.
float sht30HumidityArray[] = { 21.12, 21.12, 21.12 };		  // An array to hold the 3 most recent values.
//const char *wifiSsid = "nunya";											// Wi-Fi SSID.  Defined in privateInfo.h
//const char *wifiPassword = "nunya";										// Wi-Fi password.  Defined in privateInfo.h
//const char *broker = "nunya";												// The broker address.  Defined in privateInfo.h
//const unsigned int port = 1883;											// The broker port.


Adafruit_SHT31 sht31 = Adafruit_SHT31();
WiFiClient wifiClient;
PubSubClient mqttClient( wifiClient );

/**
 * @brief setupSht30() will initialize the SHT30 temperature and humidity sensor.
 */
void setupSht30()
{
	unsigned int address = 0x44;
	// Set to 0x45 for alternate i2c address.
	if( !sht31.begin( address ) )
	{
		Serial.printf( "Could not find SHT30 at address %X!\n", address );
		Serial.println( "  Please check the wiring and try again." );
		while( 1 )
			delay( 1 );
	}

	Serial.print( "SHT30 heater state: " );
	if( sht31.isHeaterEnabled() )
		Serial.println( "ENABLED" );
	else
		Serial.println( "DISABLED" );
} // End of the setupSht30() function.

/**
 * @brief cToF() will convert Celsius to Fahrenheit.
 */
float cToF( float value )
{
	return value * 1.8 + 32;
} // End of the cToF() function.

/**
 * @brief addValue() will add the passed value to the passed array, after moving the existing array values to higher indexes.
 */
void addValue( float valueArray[], float value )
{
	valueArray[2] = valueArray[1];
	valueArray[1] = valueArray[0];
	valueArray[0] = value;
} // End of the addValue() function.

/**
 * @brief averageArray() will return the average of values in the passed array.
 */
float averageArray( float valueArray[] )
{
	const unsigned int arraySize = 3;
	float tempValue = 0;
	for( int i = 0; i < arraySize; ++i )
	{
		tempValue += valueArray[i];
	}
	return tempValue / arraySize;
} // End of the averageArray() function.

/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
void lookupWifiCode( int code, char *buffer )
{
	switch( code )
	{
		case 0:
			snprintf( buffer, 26, "%s", "Idle" );
			break;
		case 1:
			snprintf( buffer, 26, "%s", "No SSID" );
			break;
		case 2:
			snprintf( buffer, 26, "%s", "Scan completed" );
			break;
		case 3:
			snprintf( buffer, 26, "%s", "Connected" );
			break;
		case 4:
			snprintf( buffer, 26, "%s", "Connection failed" );
			break;
		case 5:
			snprintf( buffer, 26, "%s", "Connection lost" );
			break;
		case 6:
			snprintf( buffer, 26, "%s", "Disconnected" );
			break;
		default:
			snprintf( buffer, 26, "%s", "Unknown Wi-Fi status code" );
	}
} // End of the lookupWifiCode() function.

/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
void lookupMQTTCode( int code, char *buffer )
{
	switch( code )
	{
		case -4:
			snprintf( buffer, 29, "%s", "Connection timeout" );
			break;
		case -3:
			snprintf( buffer, 29, "%s", "Connection lost" );
			break;
		case -2:
			snprintf( buffer, 29, "%s", "Connect failed" );
			break;
		case -1:
			snprintf( buffer, 29, "%s", "Disconnected" );
			break;
		case 0:
			snprintf( buffer, 29, "%s", "Connected" );
			break;
		case 1:
			snprintf( buffer, 29, "%s", "Bad protocol" );
			break;
		case 2:
			snprintf( buffer, 29, "%s", "Bad client ID" );
			break;
		case 3:
			snprintf( buffer, 29, "%s", "Unavailable" );
			break;
		case 4:
			snprintf( buffer, 29, "%s", "Bad credentials" );
			break;
		case 5:
			snprintf( buffer, 29, "%s", "Unauthorized" );
			break;
		default:
			snprintf( buffer, 29, "%s", "Unknown MQTT state code" );
	}
} // End of the lookupMQTTCode() function.

/**
 * @brief wifiBasicConnect() will connect to a SSID.
 */
void wifiBasicConnect()
{
	// Turn the LED off to show Wi-Fi is not connected.
	digitalWrite( MCU_LED, LOW );

	Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", wifiSsid );
	WiFi.mode( WIFI_STA );
	WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
	WiFi.setHostname( hostname );
	WiFi.begin( wifiSsid, wifiPassword );

	unsigned long wifiConnectionStartTime = millis();

	// Loop until connected, or until wifiConnectionTimeout.
	while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
	{
		Serial.print( "." );
		delay( 1000 );
	}
	Serial.println( "" );

	if( WiFi.status() == WL_CONNECTED )
	{
		// Print that Wi-Fi has connected.
		Serial.println( "\nWi-Fi connection established!" );
		snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
		// Turn the LED on to show that Wi-Fi is connected.
		digitalWrite( MCU_LED, HIGH );
		return;
	}
	else
		Serial.println( "Wi-Fi failed to connect in the timeout period.\n" );
} // End of the wifiBasicConnect() function.

/**
 * @brief readTelemetry() will read the telemetry and save values to global variables.
 */
void readTelemetry()
{
	rssi = WiFi.RSSI();
	// Add current readings into the appropriate arrays.
	addValue( sht30TempCArray, sht31.readTemperature() );
	addValue( sht30HumidityArray, sht31.readHumidity() );
} // End of the readTelemetry() function.

/**
 * @brief printTelemetry() will print the telemetry to the serial port.
 */
void printTelemetry()
{
	Serial.println();
	printCount++;
	Serial.printf( "Publish count %ld\n", printCount );
	Serial.printf( "MAC address: %s\n", macAddress );
	int wifiStatusCode = WiFi.status();
	char buffer[29];
	lookupWifiCode( wifiStatusCode, buffer );
	Serial.printf( "Wi-Fi status text: %s\n", buffer );
	Serial.printf( "Wi-Fi status code: %d\n", wifiStatusCode );
	if( wifiStatusCode == 3 )
	{
		Serial.printf( "IP address: %s\n", ipAddress );
		Serial.printf( "RSSI: %ld\n", rssi );
	}
	Serial.printf( "Broker: %s:%d\n", broker, port );
	int mqttStateCode = mqttClient.state();
	lookupMQTTCode( mqttStateCode, buffer );
	Serial.printf( "MQTT state: %s\n", buffer );
	Serial.printf( "SHT30 tempC: %f\n", averageArray( sht30TempCArray ) );
	Serial.printf( "SHT30 tempF: %f\n", cToF( averageArray( sht30TempCArray ) ) );
	Serial.printf( "SHT30 humidity: %f\n", averageArray( sht30HumidityArray ) );
} // End of the printTelemetry() function.

/**
 * @brief mqttCallback() will process incoming messages on subscribed topics.
 */
void publishTelemetry()
{
	char buffer[25] = "";
	snprintf( buffer, 25, "%f", averageArray( sht30TempCArray ) );
	Serial.printf( "Publishing '%s' to '%s'\n", buffer, tempCTopic );
	mqttClient.publish( tempCTopic, buffer );
	snprintf( buffer, 25, "%f", cToF( averageArray( sht30TempCArray ) ) );
	Serial.printf( "Publishing '%s' to '%s'\n", buffer, tempFTopic );
	mqttClient.publish( tempFTopic, buffer );
	snprintf( buffer, 25, "%f", averageArray( sht30HumidityArray ) );
	Serial.printf( "Publishing '%s' to '%s'\n", buffer, humidityTopic );
	mqttClient.publish( humidityTopic, buffer );
	snprintf( buffer, 25, "%ld", rssi );
	Serial.printf( "Publishing '%s' to '%s'\n", buffer, rssiTopic );
	mqttClient.publish( rssiTopic, buffer );
} // End of the printTelemetry() function.

/**
 * @brief mqttCallback() will process incoming messages on subscribed topics.
 */
void mqttCallback( char *topic, byte *payload, unsigned int length )
{
	Serial.printf( "\nMessage arrived on Topic: '%s'\n", topic );

	char message[5] = { 0x00 };

	for( unsigned int i = 0; i < length; i++ )
		message[i] = ( char ) payload[i];

	message[length] = 0x00;
	Serial.println( message );
	String str_msg = String( message );
	if( str_msg.equals( "ON" ) )
		digitalWrite( LED, HIGH );
	else if( str_msg.equals( "on" ) )
		digitalWrite( LED, HIGH );
	else if( str_msg.equals( "OFF" ) )
		digitalWrite( LED, LOW );
	else if( str_msg.equals( "off" ) )
		digitalWrite( LED, LOW );
	else
		Serial.printf( "Unknown command '%s'\n", message );
} // End of the mqttCallback() function.

/**
 * @brief mqttConnect() will connect to the MQTT broker.
 */
void mqttConnect()
{
	long time = millis();
	// Connect the first time.  Avoid subtraction overflow.  Connect after cool down.
	if( lastBrokerConnect == 0 || ( time > brokerCoolDown && ( time - brokerCoolDown ) > lastBrokerConnect ) )
	{
		lastBrokerConnect = millis();
		digitalWrite( LED, LOW );
		Serial.printf( "Connecting to broker at %s:%d...\n", broker, port );
		mqttClient.setServer( broker, port );
		mqttClient.setCallback( mqttCallback );

		// Connect to the broker, using the MAC address for a MQTT client ID.
		if( mqttClient.connect( macAddress ) )
			Serial.print( "Connected to MQTT Broker.\n" );
		else
		{
			int mqttStateCode = mqttClient.state();
			char buffer[29];
			lookupMQTTCode( mqttStateCode, buffer );
			Serial.printf( "MQTT state: %s\n", buffer );
			Serial.printf( "MQTT state code: %d\n", mqttStateCode );
			return;
		}

		mqttClient.subscribe( "led1" );
		digitalWrite( LED, HIGH );
	}
} // End of the mqttConnect() function.

/**
 * @brief setup() will configure the program.
 */
void setup()
{
	delay( 1000 );
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );
	Serial.println( "\n" );
	Serial.println( "setup() is beginning." );

	// Set GPIO 2 (MCU_LED) as an output.
	pinMode( MCU_LED, OUTPUT );
	// Turn the LED on.
	digitalWrite( MCU_LED, HIGH );

	setupSht30();

	// Set the MAC address variable to its value.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	// Read from the sensors twice, to populate their arrays.
	readTelemetry();
	readTelemetry();

	Serial.println( "Function setup() has completed." );
} // End of the setup() function.

/**
 * @brief loop() repeats over and over.
 */
void loop()
{
	if( WiFi.status() != WL_CONNECTED )
		wifiBasicConnect();
	else if( !mqttClient.connected() )
		mqttConnect();
	else
		mqttClient.loop();

	long time = millis();
	// Print the first time.  Avoid subtraction overflow.  Print every interval.
	if( lastPrintTime == 0 || ( time > printInterval && ( time - printInterval ) > lastPrintTime ) )
	{
		readTelemetry();
		printTelemetry();
		if( mqttClient.connected() )
			publishTelemetry();
		lastPrintTime = millis();

		Serial.printf( "Next print in %u seconds.\n\n", printInterval / 1000 );
	}
} // End of the loop() function.
