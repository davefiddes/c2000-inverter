# C2000 hardware helper library
add_subdirectory(driverlib)

# Board support and integration for LAUNCHXL-F28379D
add_subdirectory(device_support)

# Shared hardware device drivers for Tesla M3 inverter
add_subdirectory(inverter_drivers)

# A simple blinky
add_subdirectory(blinky)

# The inverter
add_subdirectory(inverter)

# Tesla M3 Gate Driver and PSU test application
add_subdirectory(gatedrivertest)

# Dump Tesla M3 EEPROM
add_subdirectory(dumpeeprom)
