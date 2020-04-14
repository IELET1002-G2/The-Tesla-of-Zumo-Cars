/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
//Include sensor- and servo-libraries here

BlynkTimer timer;
WebServer server;   //Port is 80 as default

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
        <td> %d C</td>\
      </tr>\
      <tr>\
        <td>VL6180X</td>\
        <td> %d cm</td>\
      </tr>\
      <tr>\
        <td>VL6180X</td>\
        <td> %d lux</td>\
      </tr>\
    </table>\
    </body>\
    </html>";

/**
 * 
*/
class Sensor {
    private:
        uint8_t inputPin;
        uint8_t dataPoints = 10;
        uint8_t sensorReadingsIndex = 0;                                    // the index of the current reading
        int sensorReadingsAverage = 0;                                      // the average
        int sensorReadingsTotal = 0;
        int sensorReadings[50];                                             // the readings from the analog input

    protected:
        /**
         * 
        */
        void setInputPin(uint8_t pinNumber) {
            inputPin = pinNumber;
            pinMode(pinNumber, INPUT);
        }

        /**
         * 
        */
        void calculateAverage(int newReading) {
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
        int read() {
            int sensorReading = analogRead(inputPin);
            calculateAverage(sensorReading);
            return sensorReading;
        }
        
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
            resetSensorReadings();
            dataPoints = value;
        }
};


class HCSR04UltrasonicSensor : public Sensor {
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
            digitalWrite(trigger, LOW);
            delayMicroseconds(5);
            digitalWrite(trigger, HIGH);
            delayMicroseconds(10);
            digitalWrite(trigger, LOW);

            int duration = pulseIn(echo, HIGH);
            int distance = duration/2*0.0343;  // The speed of sound is 343 m/s

            calculateAverage(distance);
            return distance;
        }

        //More functions related to this class
};


class TMP36TemperatureSensor : public Sensor {
    public:
        /**
         * Class constructor
        */
        TMP36TemperatureSensor(uint8_t pinNumber) {
            setInputPin(pinNumber);
        }

        //More functions related to this class
};


class VL6180XTimeOfFlight : public Sensor {
    private:

    public:
        int getDistance() {}
        int getLux() {}

        //More functions related to this class
};


HCSR04UltrasonicSensor dist(32, 33);
TMP36TemperatureSensor temp(34);
VL6180XTimeOfFlight tof;

/**
 * Slider for how many values for each point
*/
BLYNK_WRITE(V5) {
    uint8_t dataPoints = param.asInt(); // assigning incoming value from V1 to a variable 

    temp.setDataPoints(dataPoints); 
    dist.setDataPoints(dataPoints);
}

/**
 * This function sends Arduino's up time every second to Virtual Pin (5).
 * In the app, Widget's reading frequency should be set to PUSH. This means
 * that you define how often to send data to Blynk App.
 * You can send any value at any time.
 * Please don't send more that 10 values per second.
*/
void timerEvent() {
    Blynk.virtualWrite(V1, temp.read());
    Blynk.virtualWrite(V2, dist.getDistance());
    Blynk.virtualWrite(V3, temp.getAverage());
    Blynk.virtualWrite(V4, dist.getAverage());
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
char* floatToString(float number, uint8_t precision = 0) {
    static char buffer[10];
    return dtostrf(number, 0, precision, buffer);
}

/**
 * 
*/
void handleRoot() {
    char buffer[500];

    snprintf(buffer, 500, HTML, 
             dist.getAverage(), 
             temp.getAverage(),
             tof.getDistance(),
             tof.getLux());

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
}

/**
 * 
*/
void loop() {
    Blynk.run();
    timer.run();                                                    // Initiates BlynkTimer
    server.handleClient();                                          //Checking web server to manage occurring events
}

/* Other functions:


void timerEvent2() {
    // You can send any value at any time.
    // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V5, millis() / 60000);
}


// This function will be called every time widget
// in Blynk app writes values to the Virtual Pin V1
BLYNK_WRITE(V1) {
    int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
    // process received value
}


// This function is called when there is a widget
// which is requesting data from Virtual Pin V5
BLYNK_READ(V5) {
    // This command writes Arduino's uptime in seconds to Virtual Pin (5)
    Blynk.virtualWrite(V5, millis() / 1000);
}
*/
