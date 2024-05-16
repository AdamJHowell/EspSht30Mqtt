cmake_minimum_required( VERSION 3.23 )
project( EspSht30Mqtt )

set( CMAKE_CXX_STANDARD 14 )

include_directories( .
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.16/cores/esp32"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.16/libraries/ESPmDNS/src"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.16/libraries/ESPmDNS/src"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.16/libraries/WiFi/src"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.16/libraries/ArduinoOTA/src"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.16/tools/sdk/esp32/include/esp_wifi/include"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/src"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Array"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Collection"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Deserialization"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Document"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Json"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Memory"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Misc"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Serialization"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/Strings"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/ArduinoJson/src/ArduinoJson/Document"
		"C:/Users/adam.howell/Documents/Arduino/libraries/ArduinoJson/src/Adafruit_SHT31_Library"
		)

add_executable( EspSht30Mqtt
		EspSht30Mqtt.ino
		PubSubClient.h
		PubSubClient.cpp
		privateInfo.h )

set_target_properties( EspSht30Mqtt PROPERTIES LINKER_LANGUAGE CXX )
