Header  header
float32 ac_voltage       # Output Voltage in Volts (Mandatory)
float32 dc_voltage       # Input Voltage in Volts (If unmeasured NaN)
float32 load             # Current percentage load on 0 to 1 range (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range (If unmeasured NaN)
float32 temperature      # Current temperature in centigrads (If unmeasured NaN)
bool    on               # True if the inverter is on

string serial_number     # The best approximation of the battery serial number
