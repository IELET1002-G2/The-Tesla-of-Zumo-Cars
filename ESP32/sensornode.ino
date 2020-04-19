/**
 * Sensornode pin chart:
 * AlarmSystem: LED1: 18, LED2: 19, Buzzer: 16
 * VL6180X: I2C: SDAPin: 21, SCLPin: 22
 * Servo: 26
 * Ultrasonic: TrigPin: 32, EchoPin: 33
 * TMP36: 34
 */

/**
 * Blynk virtual pin chart
 * V0   Set data points in average calculations
 * V1   TMP36: Momentary temperature reading
 * V2   TMP36: Average temperature reading
 * V3   HCSR04: Momentary distance reading
 * V4   HCSR04: Average distance reading
 * V5   VL6180X: Momentary distance reading
 * V6   VL6180X: Average distance reading
 * V7   VL6180X: Momentary lux reading
 * V8   VL6180X: Average lux reading
 * V9   TMP36: Maxima temperature reading
 * V10  TMP36: Minima temperature reading
 * V11  HCSR04: Maxima distance reading
 * V12  HCSR04: Minima distance reading
 * V13  VL6180X: Maxima distance reading
 * V14  VL6180X: Minima distance reading
 * V15  VL6180X: Maxima lux reading
 * V16  VL6180X: Minima lux reading
 * V17  Terminal
 * V18  Servo
 * V19  Alarm LED
*/

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Adafruit_VL6180X.h>
#include <EasyBuzzer.h>

BlynkTimer timer;
WebServer server;   //Port is 80 as default
Adafruit_VL6180X vl;
Servo servo;

int vbuttonState;  //ONLY FOR TESTIN. REMOVE LATER WHEN NOT NEEDED ANYMORE.

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
        T maxValue = 0;
        T minValue = 10000;

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

            if (newReading > maxValue) maxValue = newReading;
            if (newReading < minValue) minValue = newReading;
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
        int getMax() {
            return maxValue;
        }

        /**
         * 
         */
        int getMin() {
            return minValue;
        }

        /**
         * 
        */
        void resetSensorReadings() {
            for (uint8_t i = 0; i < 50; i++) sensorReadings[i] = 0;
            sensorReadingsTotal = 0;
        }

        /**
         * 
         */
        void resetValuesExtrema() {
            maxValue = 0;
            minValue = 10000;
        }

        /**
         * 
        */
        void setDataPoints(uint8_t value) {
            if (value > dataPoints) {
                for (uint8_t i = dataPoints - 1; i < value; i++) {
                    sensorReadingsTotal += sensorReadingsAverage;
                    sensorReadings[i] = sensorReadingsAverage;
                }
            } else {
                for (uint8_t i = value; i < 50; i++) {
                    sensorReadingsTotal -= sensorReadings[i];
                    sensorReadings[i] = 0;     // Only deletes data that is not overwritten after data point change
                }
            }
            dataPoints = value;
        }
};

/**
 * 
 */
class alarmSystem {
    private:
        int alarmLED1;                                                      //Two red alarm LEDs should blink alternately, while alarm sound from buzzer
        int alarmLED2;
        int alarmBuzzer;
        bool toggleAlarmState;

    public:
        /**
         * 
         */
        alarmSystem(int Led1Pin, int Led2Pin, int buzzerPin) {
            pinMode(Led1Pin, OUTPUT);
            pinMode(Led2Pin, OUTPUT);
            pinMode(buzzerPin, OUTPUT);

            alarmLED1 = Led1Pin;
            alarmLED2 = Led2Pin;
            EasyBuzzer.setPin(buzzerPin);
        }

    /**
     * 
     */
    void alarm() {

        if (toggleAlarmState) {
            digitalWrite(alarmLED1, HIGH);
            digitalWrite(alarmLED2, LOW);
            EasyBuzzer.beep(1000);                  
            toggleAlarmState = !toggleAlarmState;
        }

        else if (!toggleAlarmState) {
            digitalWrite(alarmLED1, LOW);
            digitalWrite(alarmLED2, HIGH);
            EasyBuzzer.beep(800);                     
            toggleAlarmState = !toggleAlarmState;
        }
    }

    /**
     * 
     */
    void resetAlarm() {
        digitalWrite(alarmLED1, LOW);                               //Set LEDs low and stop buzzer                    
        digitalWrite(alarmLED2, LOW);
        EasyBuzzer.beep(0);                                         
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
            float rawValue = analogRead(inputPin);
            float temperature = (rawValue - 500) / 10;
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
alarmSystem alarmSystem(18, 19, 16);
WidgetLED BlynkAlarmLED(V19);

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
BLYNK_WRITE(V18) {
    servo.write(param.asInt()); //writes value from blynk slider to servo
}

//FOR TESTING PURPOSES ONLY
BLYNK_WRITE(V20) {
    vbuttonState = param.asInt();
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
void extremaUpdate() {
    Blynk.virtualWrite(V9, temp.getMax());                          //Pushes all max/min values every 30 s           
    Blynk.virtualWrite(V10, temp.getMin());
    Blynk.virtualWrite(V11, dist.getMax());
    Blynk.virtualWrite(V12, dist.getMin());
    Blynk.virtualWrite(V13, vlDist.getMax());
    Blynk.virtualWrite(V14, vlDist.getMin());
    Blynk.virtualWrite(V15, vlLux.getMax());
    Blynk.virtualWrite(V16, vlLux.getMin());

    temp.resetValuesExtrema();                                      //Resets all extrema values
    dist.resetValuesExtrema();
    vlDist.resetValuesExtrema();
    vlLux.resetValuesExtrema();
}

/**
 * 
 */
void timerEventToggleAlarm() {

    static bool resetAlarmOnce;                                     //Interlock to keep buzzer making tikking noise from reset every time timerEventToggleAlarm is called

    if(vbuttonState) { //ONLY FOR TESTING. REMOVE LATER AND UNCOMMET NEXT LINE.

    //if (alarmTrigger()) {                                           //Will go high as soon as two or more alarm levels is reached
        alarmSystem.alarm();                                        //Activate alarm system
        BlynkAlarmLED.on();
        resetAlarmOnce = true;
    }

    if(!vbuttonState && resetAlarmOnce) { //ONLY FOR TESTING. REMOVE LATER AND UNCOMMENT NEXT LINE.
    
    //if (!alarmTrigger() && resetAlarmOnce) {                        //Will reset alarm when false, but may be a bit slow, due to sensor values new every 30 s
        alarmSystem.resetAlarm();                                   //Reset alarm system
        BlynkAlarmLED.off();
        resetAlarmOnce = false;
    }
}

/**
 * 
*/
BLYNK_APP_CONNECTED() {
    Blynk.virtualWrite(V0, 10);                                     // Resets slider to 10 when app starts
}

/**
 * 
*/
void handleRoot() {
    char buffer[500];
    float temperature = temp.getAverage();
    float lux = vlLux.getAverage();
    int distance1 = vlDist.getAverage();
    int distance2 = dist.getAverage();

    snprintf(buffer, 500, HTML, 
             distance2, 
             temperature,
             distance1,
             lux);

    server.send(200, "text/html", buffer);                          //Code 200, display HTML code.
}

/**
 * 
 */
bool alarmTrigger() {
    static bool ultrasonicAlarm;
    static bool temperatureAlarm;
    static bool timeOfFlightAlarm;
    static bool luxAlarm;

    if (dist.getMin() < 15) ultrasonicAlarm = true;                 //If object is closer than 15 cm, return true
    else ultrasonicAlarm = false;

    if (temp.getMax() > 25.0) temperatureAlarm = true;              //If tempretaure is greater than 25.0 degrees celsius, return true
    else temperatureAlarm = false;

    if (vlDist.getMin() < 75) timeOfFlightAlarm = true;             //If object is closar than 75 mm, return true
    else timeOfFlightAlarm = false;

    if (vlLux.getMax() > 1000.0) luxAlarm = true;                   //If lux level is greater than 1000.0 lux, return true
    else luxAlarm = false;

    if (
        ultrasonicAlarm   && temperatureAlarm  ||                   //Every scenario is covered, returns true if two or more alarm limits is reached
        ultrasonicAlarm   && timeOfFlightAlarm ||
        ultrasonicAlarm   && luxAlarm          ||
        temperatureAlarm  && timeOfFlightAlarm ||
        temperatureAlarm  && luxAlarm          ||
        timeOfFlightAlarm && luxAlarm
    ) return true;
    else return false;
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

    timer.setInterval(100L, timerEventToggleAlarm);                 //Timer that should toggle alarm LED state and buzzer pitch when alarm
    timer.setInterval(1000L, timerEvent);                           // Setup a function to be called every second and minute
    timer.setInterval(30000L, extremaUpdate);                       //Timer that pushes max/min values every 30 s interval

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
    EasyBuzzer.update();
}
