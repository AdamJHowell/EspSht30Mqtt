cmake_minimum_required( VERSION 3.23 )
project( EspSht30Mqtt )

set( CMAKE_CXX_STANDARD 14 )

include_directories( .
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.7/cores/esp32"
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.7/libraries/ESPmDNS/src"
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.7/libraries/WiFi/src"
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.7/libraries/ArduinoOTA/src"
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.7/tools/sdk/esp32/include/esp_wifi/include"
		"C:/Code/Arduino/libraries"
		"C:/Code/Arduino/libraries/ArduinoJson"
		"C:/Code/Arduino/libraries/ArduinoJson/src"
		"C:/Code/Arduino/libraries/ArduinoJson/Array"
		"C:/Code/Arduino/libraries/ArduinoJson/Collection"
		"C:/Code/Arduino/libraries/ArduinoJson/Deserialization"
		"C:/Code/Arduino/libraries/ArduinoJson/Document"
		"C:/Code/Arduino/libraries/ArduinoJson/Json"
		"C:/Code/Arduino/libraries/ArduinoJson/Memory"
		"C:/Code/Arduino/libraries/ArduinoJson/Misc"
		"C:/Code/Arduino/libraries/ArduinoJson/Serialization"
		"C:/Code/Arduino/libraries/ArduinoJson/Strings"
		"C:/Code/Arduino/libraries/ArduinoJson/src/ArduinoJson/Document"
		"C:/Code/Arduino/libraries/Adafruit_SHT31_Library"
		)

add_executable( EspSht30Mqtt
		EspSht30Mqtt.ino
		PubSubClient.h
		PubSubClient.cpp
		privateInfo.h )

set_target_properties( EspSht30Mqtt PROPERTIES LINKER_LANGUAGE CXX )
