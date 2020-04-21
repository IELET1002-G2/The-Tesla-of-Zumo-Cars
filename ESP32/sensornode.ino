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
template<typename T>                                                // Class Template to make sensor class accept several data types
class SensorData {
    private:
        uint8_t dataPoints = 10;
        uint8_t writeIndex = 0;                                     // The index of the current reading
        T sensorReadings[50];                                       // The readings from the analog input
        T maxReading = 0;
        T minReading = 10000;

    protected:
        /**
         * 
        */
        void addDataPoint(T newReading) {                           // Method that takes placeholder T for the data type used as argument
            sensorReadings[writeIndex] = newReading;                // Read from the sensor

            writeIndex++;                                           // Advance to the next position in the array:
            if (writeIndex >= 50) writeIndex = 0;                   // If we're at the end of the array wrap around to the beginning

            if (newReading > maxReading) maxReading = newReading;   // Update the maxReading if new maximum is reached
            if (newReading < minReading) minReading = newReading;   // Update the minReading if new minimum is reached
        }

    public:
        /**
         * 
        */
        T getAverage() {
            T sum = 0;
            for (uint8_t pointsBack = 1; pointsBack <= dataPoints; pointsBack++) {
                int8_t readIndex = writeIndex - pointsBack;
                if (readIndex < 0) readIndex += 50;

                sum += sensorReadings[readIndex];
            }
            return sum / dataPoints;
        }

        /**
         * 
         */
        int getMax() {                                              // Method that returns maxReading       
            return maxReading;
        }

        /**
         * 
         */
        int getMin() {                                              // Method that returns minReading
            return minReading;
        }

        /**
         * 
        */
        void resetSensorReadings() {                                // Method that resets the sensor reading array
            for (uint8_t i = 0; i < 50; i++) sensorReadings[i] = 0;
        }

        /**
         * 
         */
        void resetValuesExtrema() {                                 // Method that resets the extrema values
            maxReading = 0;
            minReading = 10000;
        }

        /**
         * 
        */
        void setDataPoints(uint8_t value) {                         // Method to set data points to specified value
            dataPoints = value;
        }
};

/**
 * 
 */
class alarmSystem {
    private:
        int alarmLED1;                                              // Declare variables for use in class
        int alarmLED2;
        int alarmBuzzer;
        bool toggleAlarmState;

    public:
        /**
         * 
         */
        alarmSystem(int Led1Pin, int Led2Pin, int buzzerPin) {      // Two red alarm LEDs should blink alternately, while alarm sound from buzzer
            pinMode(Led1Pin, OUTPUT);
            pinMode(Led2Pin, OUTPUT);

            alarmLED1 = Led1Pin;
            alarmLED2 = Led2Pin;
            EasyBuzzer.setPin(buzzerPin);
        }

    /**
     * 
     */
    void alarm() {

        if (toggleAlarmState) {                                     // State machine to toggle between LEDs and buzzer pitch
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
        digitalWrite(alarmLED1, LOW);                               // Set LEDs low and stops buzzer                    
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

            int duration = pulseIn(echo, HIGH, 25000);              // Timeout needs to be ~ 25 ms long to get full range of sensor (up to four meters)
            int distance = duration/2*0.0343;                       // The speed of sound is 343 m/s

            addDataPoint(distance);
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
            float voltage = rawValue * 3300.0 / 4095.0;
            float temperature = (voltage - 500) / 10;

            addDataPoint(temperature);
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

            addDataPoint(distance);
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

            addDataPoint(lux);
            return lux;
        }
};


HCSR04UltrasonicSensor dist(32, 33);                                // Create instances of classes
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
    uint8_t dataPoints = param.asInt();                             // Assigning incoming value from V1 to a variable 

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
    servo.write(param.asInt());                                     // Writes value from blynk slider to servo
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
    Blynk.virtualWrite(V9, temp.getMax());                          // Pushes all max/min values every 30 s           
    Blynk.virtualWrite(V10, temp.getMin());
    Blynk.virtualWrite(V11, dist.getMax());
    Blynk.virtualWrite(V12, dist.getMin());
    Blynk.virtualWrite(V13, vlDist.getMax());
    Blynk.virtualWrite(V14, vlDist.getMin());
    Blynk.virtualWrite(V15, vlLux.getMax());
    Blynk.virtualWrite(V16, vlLux.getMin());

    temp.resetValuesExtrema();                                      // Resets all extrema values
    dist.resetValuesExtrema();
    vlDist.resetValuesExtrema();
    vlLux.resetValuesExtrema();
}

/**
 * 
 */
bool alarmTrigger() {
    bool ultrasonicAlarm = dist.getMin() < 15;                      // If object is closer than 15 cm, return true
    bool temperatureAlarm = temp.getMax() > 25.0;                   // If tempretaure is greater than 25.0 degrees celsius, return true
    bool timeOfFlightAlarm = vlDist.getMin() < 75;                  // If object is closer than 75 mm, return true
    bool luxAlarm = vlLux.getMax() > 1000.0;                        // If lux level is greater than 1000.0 lux, return true

    return
        ultrasonicAlarm   && temperatureAlarm  ||                   // Every scenario is covered, returns true if two or more alarm limits is reached
        ultrasonicAlarm   && timeOfFlightAlarm ||
        ultrasonicAlarm   && luxAlarm          ||
        temperatureAlarm  && timeOfFlightAlarm ||
        temperatureAlarm  && luxAlarm          ||
        timeOfFlightAlarm && luxAlarm;
}

/**
 * 
 */
void timerEventToggleAlarm() {

    static bool resetAlarmOnce;                                     // Interlock to keep buzzer making tikking noise from reset every time timerEventToggleAlarm is called

    if(vbuttonState) { //ONLY FOR TESTING. REMOVE LATER AND UNCOMMET NEXT LINE.

    //if (alarmTrigger()) {                                           // Will go high as soon as two or more alarm levels is reached
        alarmSystem.alarm();                                        // Activate alarm system
        BlynkAlarmLED.on();
        resetAlarmOnce = true;
    }

    if(!vbuttonState && resetAlarmOnce) { //ONLY FOR TESTING. REMOVE LATER AND UNCOMMENT NEXT LINE.
    
    //if (!alarmTrigger() && resetAlarmOnce) {                        // Will reset alarm when false, but may be a bit slow, due to sensor values new every 30 s
        alarmSystem.resetAlarm();                                   // Reset alarm system
        BlynkAlarmLED.off();
        resetAlarmOnce = false;
    }
}

/**
 * 
*/
BLYNK_WRITE(V19){                            //Test button for servo alarm
  int servoTestButton = (param.asInt());     
  if (servoTestButton == 1) {                
    timer.setTimer(1000L, servo1, 1);        
  }
  else {
    servo.write(0);                          //if button is not pressed go to 0 degrees
  }
}

/**
 * 
*/
void servo1() {                              // function for sweep towards 180 degrees
  servo.write(180);                          
  timer.setTimer(1000L, servo2, 1);
}

/**
 * 
*/
void servo2() {                              // function for sweep towards 0 degrees
  servo.write(0);
  Blynk.syncVirtual(V19);                    // checks if button is still pressed or not
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

    server.send(200, "text/html", buffer);                          // Code 200, display HTML code.
}

/**
 * 
*/
void setup() {
    Serial.begin(115200);                                           // Debug console
    Blynk.begin(AUTH, SSID, PASS, IPAddress(91,192,221,40), 8080);
    server.begin();                                                 // Initiates web server.
    server.on("/", handleRoot);                                     // Manage HTTP request and run handleRoot() when "IP"/ is searched in browser.
    Serial.println("HTTP server started");

    timer.setInterval(500L, timerEventToggleAlarm);                 // Timer that should toggle alarm LED state and buzzer pitch when alarm
    timer.setInterval(1000L, timerEvent);                           // Setup a function to be called every second and minute
    timer.setInterval(30000L, extremaUpdate);                       // Timer that pushes max/min values every 30 s interval

    dist.resetSensorReadings();                                     // Initialize all the readings to 0
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
    server.handleClient();                                          // Checking web server to manage occurring events
    EasyBuzzer.update();
}
