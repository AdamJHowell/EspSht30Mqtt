cmake_minimum_required( VERSION 3.23 )
project( EspSht30Mqtt )

set( CMAKE_CXX_STANDARD 14 )

add_executable( EspSht30Mqtt
		EspSht30Mqtt.ino
		privateInfo.h )

set_target_properties( EspSht30Mqtt PROPERTIES LINKER_LANGUAGE CXX )
