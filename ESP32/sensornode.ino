/**
 * V0   Set data points in average calculations
 * V1   TMP36: Momentary temperature reading
 * V2   TMP36: Average temperature reading
 * V3   HCSR04: Momentary distance reading
 * V4   HCSR04: Average distance reading
 * V5   VL6180X: Momentary distance reading
 * V6   VL6180X: Average distance reading
 * V7   VL6180X: Momentary lux reading
 * V8   VL6180X: Average lux reading
 * V9   Terminal
 * V10  Servo
*/

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Adafruit_VL6180X.h>

BlynkTimer timer;
WebServer server;   //Port is 80 as default
Adafruit_VL6180X vl;
Servo servo;

const char* AUTH = "";
const char* SSID = "";
const char* PASS = "";
const char* HTML = 
    "<!DOCTYPE html>\
    <html>\
    <body>\
    <h1>Gruppe 2: Web server</h1>\
    <h2>Table of sensor values</h2>\
    <table>\
      <tr>\
        <th>Sensor</th>\
        <th>Value</th>\
      </tr>\
      <tr>\
        <td>HCSR04</td>\
        <td> %d cm</td>\
      </tr>\
      <tr>\
        <td>TMP36T</td>\
        <td> %0.1f C</td>\
      </tr>\
      <tr>\
        <td>VL6180X</td>\
        <td> %d mm</td>\
      </tr>\
      <tr>\
        <td>VL6180X</td>\
        <td> %0.1f lux</td>\
      </tr>\
    </table>\
    </body>\
    </html>";

/**
 * 
*/
template<typename T>                                                      //Class Template to make sensor class accept several data types
class SensorData {
    private:
        uint8_t inputPin;
        uint8_t dataPoints = 10;
        uint8_t sensorReadingsIndex = 0;                                  // the index of the current reading
        T sensorReadingsAverage = 0;                                      // the average
        T sensorReadingsTotal = 0;
        T sensorReadings[50];                                             // the readings from the analog input

    protected:
        /**
         * 
        */
        void calculateAverage(T newReading) {                               //Method that takes placeholder T for the data type used as argument
            sensorReadingsTotal -= sensorReadings[sensorReadingsIndex];     // subtract the last reading
            sensorReadings[sensorReadingsIndex] = newReading;               // read from the sensor
            sensorReadingsTotal += newReading;                              // add the reading to the total
            sensorReadingsAverage = sensorReadingsTotal/dataPoints;         // calculate the average

            sensorReadingsIndex++;                                          // advance to the next position in the array:
            if (sensorReadingsIndex >= dataPoints) sensorReadingsIndex = 0; // if we're at the end of the array wrap around to the beginning
        }

    public:
        /**
         * 
        */
        int getAverage() {
            return sensorReadingsAverage;
        }

        /**
         * 
        */
        void resetSensorReadings() {
            for (uint8_t i = 0; i < 50; i++) sensorReadings[i] = 0;
        }

        /**
         * 
        */
        void setDataPoints(uint8_t value) {
            for (uint8_t i = value; i < 50; i++) sensorReadings[i] = 0;     // Only deletes data that is not overwritten after data point change
            dataPoints = value;
        }
};

/**
 * 
*/
class HCSR04UltrasonicSensor : public SensorData<int> {
    private:
        uint8_t trigger;
        uint8_t echo;

    public:
        /**
         * Class constructor
        */
        HCSR04UltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
            pinMode(trigPin, OUTPUT);
            pinMode(echoPin, INPUT);
            
            trigger = trigPin;
            echo = echoPin;
        }

        /**
         * 
        */
        int getDistance() {
            digitalWrite(trigger, HIGH);
            delayMicroseconds(10);
            digitalWrite(trigger, LOW);

            int duration = pulseIn(echo, HIGH);
            int distance = duration/2*0.0343;  // The speed of sound is 343 m/s

            calculateAverage(distance);
            return distance;
        }
};

/**
 * 
*/
class TMP36TemperatureSensor : public SensorData<float> {
    private:
        uint8_t inputPin;

    public:
        /**
         * Class constructor
        */
        TMP36TemperatureSensor(uint8_t pinNumber) {
            pinMode(pinNumber, INPUT);
            inputPin = pinNumber;  
        }

        /**
         * 
        */
        float getTemperature() {
            float temperature = analogRead(inputPin);
            //Magic happens, temp processed...
            calculateAverage(temperature);
            return temperature;
        }
};

/**
 * 
*/
class VL6180XRangeSensor : public SensorData<int> {
    private:
        Adafruit_VL6180X* vlPointer;

    public:
        /**
         * Class constructor
        */
        VL6180XRangeSensor(Adafruit_VL6180X* instancePointer) {
            vlPointer = instancePointer;
        }
        /**
         * 
        */
        uint8_t getDistance() {
            uint8_t distance;
            if (vlPointer->begin()) distance = vlPointer->readRange();  //Accessing members of instance without manually deferencing pointer

            calculateAverage(distance);
            return distance;
        }
};

/**
 * 
*/
class VL6180XLuxSensor : public SensorData<float> {
    private:
        Adafruit_VL6180X* vlPointer;

    public:
        /**
         * Class constructor
        */
        VL6180XLuxSensor(Adafruit_VL6180X* instancePointer) {
            vlPointer = instancePointer;
        }

        /**
         * 
        */
        float getLux() {
            float lux;
            if (vlPointer->begin()) lux = vlPointer->readLux(VL6180X_ALS_GAIN_5);

            calculateAverage(lux);
            return lux;
        }
};


HCSR04UltrasonicSensor dist(32, 33);
TMP36TemperatureSensor temp(34);
VL6180XRangeSensor vlDist(&vl);
VL6180XLuxSensor vlLux(&vl);
WidgetTerminal terminal(V9);

/**
 * Slider for how many values for each point
*/
BLYNK_WRITE(V0) {
    uint8_t dataPoints = param.asInt(); // assigning incoming value from V1 to a variable 

    temp.setDataPoints(dataPoints); 
    dist.setDataPoints(dataPoints);
    vlDist.setDataPoints(dataPoints); 
    vlLux.setDataPoints(dataPoints);
}

/**
 * 
*/
BLYNK_WRITE(V9) {
    terminal.println(temp.getTemperature());
    terminal.println(dist.getDistance());
    terminal.println(vlDist.getDistance());
    terminal.println(vlLux.getLux());
    terminal.flush();
}

/**
 * 
*/
BLYNK_WRITE(V10) {
    servo.write(param.asInt()); //writes value from blynk slider to servo
}

/**
 * This function sends Arduino's up time every second to Virtual Pin (5).
 * In the app, Widget's reading frequency should be set to PUSH. This means
 * that you define how often to send data to Blynk App.
 * You can send any value at any time.
 * Please don't send more that 10 values per second.
*/
void timerEvent() {
    Blynk.virtualWrite(V1, temp.getTemperature());
    Blynk.virtualWrite(V2, temp.getAverage());
    Blynk.virtualWrite(V3, dist.getDistance());
    Blynk.virtualWrite(V4, dist.getAverage());
    Blynk.virtualWrite(V5, vlDist.getDistance());
    Blynk.virtualWrite(V6, vlDist.getAverage());
    Blynk.virtualWrite(V7, vlLux.getLux());
    Blynk.virtualWrite(V8, vlLux.getAverage());
}

/**
 * 
*/
BLYNK_APP_CONNECTED() {
    Blynk.virtualWrite(V0, 10); // Resets slider to 10 when app starts
}

/**
 * 
*/
void serverUpdate() {
    handleRoot();
}

/**
 * 
*/
void handleRoot() {
    char buffer[500];

    snprintf(buffer, 500, HTML, 
             dist.getAverage(), 
             temp.getAverage(),
             vlDist.getAverage(),
             vlLux.getAverage());

    server.send(200, "text/html", buffer);                            //Code 200, display HTML code.
}

/**
 * 
*/
void setup() {
    Serial.begin(115200);                                           // Debug console
    Blynk.begin(AUTH, SSID, PASS, IPAddress(91,192,221,40), 8080);
    server.begin();                                                 //Initiates web server.
    server.on("/", handleRoot);                                     //Manage HTTP request and run handleRoot() when "IP"/ is searched in browser.
    Serial.println("HTTP server started");

    timer.setInterval(1000L, timerEvent);                           // Setup a function to be called every second and minute
    timer.setInterval(60000L, serverUpdate);

    dist.resetSensorReadings();                                     // initialize all the readings to 0:
    temp.resetSensorReadings();
    vlDist.resetSensorReadings();
    vlLux.resetSensorReadings();

    terminal.clear(); 
    terminal.println(F("blynk program running"));
    terminal.flush();
}

/**
 * 
*/
void loop() {
    Blynk.run();
    timer.run();                                                    // Initiates BlynkTimer
    server.handleClient();                                          //Checking web server to manage occurring events
}
