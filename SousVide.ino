#include <OneWire.h>

/*

    SousVide
    Copyright (c) 2016 Max Vilimpoc

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
	
*/

// Based on the OneWire example, and so on.

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library

enum Pin
{
    ONE_WIRE_BUS_PIN     = 5,

    HEATING_RELAY_PIN    = 8,
    WATER_PUMP_RELAY_PIN = 9,
};

enum
{
    START_CONVERSION   = 0x44,
    CONVERSION_TIMEOUT = 1000,

    DATA_LENGTH        = 32
};

enum SensorType
{
    DS18S20,
    DS18B20,
    DS1822
};

struct Sensor
{
    const char * const name;
    const uint8_t      address[8];

    const SensorType   type;

    float const        Thistory[DATA_LENGTH];
    float              Tcurrent;  // Current temperature
    float              Trate;     // Rate of change of the temperature
};

static Sensor sensors[] = {
    { "1: Water Temperature",     {0x28, 0xFF, 0x8E, 0x8A, 0x64, 0x14, 0x03, 0xED}, DS18B20, {0}, 0.0, 0.0 },
    { "2: Hot Plate Temperature", {0x28, 0xFF, 0x2A, 0x99, 0x64, 0x14, 0x03, 0x71}, DS18B20, {0}, 0.0, 0.0 },
    { "3: Ambient Temperature",   {0x28, 0xFF, 0x22, 0xE4, 0x64, 0x14, 0x01, 0xC6}, DS18B20, {0}, 0.0, 0.0 }
};

static Sensor& waterTemperature(sensors[0]);
static float   waterTemperatureSetPoint;

//struct Actuator
//{
//    const char *   const name;
//    const Pin            pin;
//    
//    const Sensor * const sensor;  //
//
//    int16_t setPoint;  // Temperature to achieve.
//
//    int16_t error;
//    int16_t errorSum;
//    int16_t errorDifference;
//
//    // virtual bool goNoGo();
//};
//
//static Actuator actuators[] = {
//    { 
//        .name     = "1: Hot Plate Relay", 
//        .pin      = HEATING_RELAY_PIN,
//        .sensor   = &sensors[2],
//        .setPoint = 0, 
//        .error    = 0, 
//        .errorSum = 0,
//        .errorDifference = 0
//    },
//    {
//        .name     = "2: Water Pump Relay",
//        .pin      = WATER_PUMP_RELAY_PIN,
//        .sensor   = &sensors[1],
//        
//    }
//};
//

class Actuator
{
    const String name;
    const Pin    pin;
public:
    void on(void)
    {
        digitalWrite(pin, HIGH);
    }
    
    void off(void)
    {
        digitalWrite(pin, LOW);
    }

    Actuator(String const& name, Pin pin) : 
        name(name),
        pin(pin)
    {
        pinMode(pin, OUTPUT);
        off();
    }
};

Actuator heater("Hot Plate Relay",  HEATING_RELAY_PIN);
Actuator pump  ("Water Pump Relay", WATER_PUMP_RELAY_PIN);

static const uint8_t SENSORS_COUNT = sizeof(sensors) / sizeof(Sensor);

static OneWire bus(ONE_WIRE_BUS_PIN);  // 4.7K pull-up resistor is necessary.
static Stream& console(Serial);

static void updateSensors(void)
{
    byte present = 0;
    byte data[12];
    float celsius, fahrenheit;
    
    for (auto sensor : sensors)
    {
        console.print(F("Sensor "));
        console.println(sensor.name);

        bus.reset();
        bus.select(sensor.address);
        bus.write(START_CONVERSION, 1);        // start conversion, with parasite power on at the end

        delay(CONVERSION_TIMEOUT);     // maybe 750ms is enough, maybe not
        // we might do a bus.depower() here, but the reset will take care of it.

        present = bus.reset();
        bus.select(sensor.address);    
        bus.write(0xBE);         // Read Scratchpad

        console.print("  Data = ");
        console.print(present, HEX);
        console.print(" ");

        for (uint8_t i = 0; i < 9; i++) 
        {           
            // we need 9 bytes
            data[i] = bus.read();
            console.print(data[i], HEX);
            console.print(" ");
        }

        console.print(" CRC=");
        console.print(OneWire::crc8(data, 8), HEX);
        console.println();

        // Convert the data to actual temperature
        // because the result is a 16 bit signed integer, it should
        // be stored to an "int16_t" type, which is always 16 bits
        // even when compiled on a 32 bit processor.
        int16_t raw = (data[1] << 8) | data[0];

        if (DS18S20 == sensor.type)
        {
            raw = raw << 3; // 9 bit resolution default
            if (data[7] == 0x10) 
            {
                // "count remain" gives full 12 bit resolution
                raw = (raw & 0xFFF0) + 12 - data[6];
            }
        } 
        else
        {
            byte cfg = (data[4] & 0x60);
    
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
        }
        
        celsius    = (float) raw / 16.0;
        fahrenheit = celsius * 1.8 + 32.0;

        float newRate = waterTemperature.Trate * 2;
        newRate += celsius - waterTemperature.Tcurrent;
        newRate /= 3.0;
        waterTemperature.Trate = newRate;

        waterTemperature.Tcurrent = celsius;
        
        console.print("  Temperature = ");
        console.print(raw);
        console.print(" Raw, ");
        console.print(celsius);
        console.print(" Celsius, ");
        console.print(fahrenheit);
        console.println(" Fahrenheit");
    }
}

static void updateActuators(void)
{
    float error = waterTemperatureSetPoint - waterTemperature.Tcurrent;


    // Naive implementation.
    if (error > 0)
    {
        // Check the rate at which the temperature is closing in on the set point.
        //
        // Because the heating element will continue to pump energy into the water
        // after it is turned off, it needs to be turned off ahead of us hitting
        // the set point.
        //
        // But we need to track the heating element performance, and tune the control
        // system to minimize the turn off time.
        //
        // Print an estimate of the time to reach the set point.
        heater.on();
    }
    else
    {
        // Stop heat.
        heater.off();
    }

    // Project the temperature into the future, using calculated rate of change.

    // 

    
}

void setup(void)
{
    Serial.begin(115200);

    //
    waterTemperatureSetPoint = 65;
}

void loop(void)
{
//    if (!bus.search(address))
//    {
//        console.println("No more addresses.");
//        console.println();
//        bus.reset_search();
//        delay(250);
//        return;
//    }
//    
//    console.print("ROM =");
//    for (i = 0; i < 8; i++)
//    {
//        console.write(' ');
//        console.print(address[i], HEX);
//    }
//
//    if (OneWire::crc8(address, 7) != address[7])
//    {
//        console.println("CRC is not valid!");
//        return;
//    }
//
//    console.println();
//
//    // the first ROM byte indicates which chip
//    switch (address[0]) 
//    {
//    case 0x10:
//        console.println("  Chip = DS18S20");  // or old DS1820
//        type_s = 1;
//        break;
//    case 0x28:
//        console.println("  Chip = DS18B20");
//        type_s = 0;
//        break;
//    case 0x22:
//        console.println("  Chip = DS1822");
//        type_s = 0;
//        break;
//    default:
//        console.println("Device is not a DS18x20 family device.");
//        return;
//    }

    updateSensors();
    updateActuators();
}
