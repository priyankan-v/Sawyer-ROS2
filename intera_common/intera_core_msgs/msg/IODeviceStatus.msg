## IO Device status
builtin_interfaces/Time time     # time the message was created
IOComponentStatus device   # device status data
IODataStatus[]    ports    # Ports status
IODataStatus[]    signals  # Signals status
builtin_interfaces/Time[] commands # recent command timestamps, for syncing
string[]          responses # recent command responses

