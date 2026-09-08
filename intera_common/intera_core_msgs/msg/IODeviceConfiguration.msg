## IO Device Configuration
builtin_interfaces/Time time          # configuration command timestamp
string commanded                      # configuration command JSON
string upgraded                       # configuration command JSON, upgraded to current schema revision
IOComponentConfiguration   device     # instantiated device configuration
IOComponentConfiguration[] ports      # instantiated port configurations
IOComponentConfiguration[] signals    # instantiated signal configurations
