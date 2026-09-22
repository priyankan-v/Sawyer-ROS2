bool homed
bool ready
bool enabled
bool stopped
bool error
bool low_voltage

uint8 estop_button

uint8 ESTOP_BUTTON_UNPRESSED=0
uint8 ESTOP_BUTTON_PRESSED=1
uint8 ESTOP_BUTTON_UNKNOWN=2
uint8 ESTOP_BUTTON_RELEASED=3

uint8 estop_source

uint8 ESTOP_SOURCE_NONE=0
uint8 ESTOP_SOURCE_USER=1
uint8 ESTOP_SOURCE_UNKNOWN=2
uint8 ESTOP_SOURCE_FAULT=3
uint8 ESTOP_SOURCE_ENGINE=4