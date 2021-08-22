#include <Arduino.h>
#include <TinyGPS++.h>

extern TinyGPSPlus gps;

[[noreturn]] void taskGPS(void *parameter);
