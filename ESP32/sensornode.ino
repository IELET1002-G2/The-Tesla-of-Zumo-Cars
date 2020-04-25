/**
 * Sensornode pin chart:
 * AlarmSystem: LED1: 18, LED2: 19, Buzzer: 17
 * VL6180X: I2C: SDAPin: 21, SCLPin: 22
 * Servo: 16
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
 * V18  Blynk Servo Slider
 * V19  Blynk Servo Test button
 * V20  Blynk Alarm LED
*/

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Adafruit_VL6180X.h>

BlynkTimer timer;
WebServer server;           // Port is 80 as default
Adafruit_VL6180X vl;
Servo servo;

int vbuttonState;  //ONLY FOR TESTIN. REMOVE LATER WHEN NOT NEEDED ANYMORE.

const char* AUTH = "";      // Auth token from Blynk app
const char* SSID = "";      // Network name
const char* PASS = "";      // Network password
const char* HTML =          // HTML code to be run on web server
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
 * Class template for storing and processing data read from sensors. Contains
 * methods for adding, manipulating and reading data. Should preferably be 
 * inherited by derived sensor class, but inheritance can be omitted. Should always
 * be constructed using explicit arguments, 
 * e.g. class derived : public SensorData<int> or SensorData<int> object;
 * 
 * @param T type of numbers to be processed, e.g. int and float
*/
template<typename T>
class SensorData {
    private:
        uint8_t dataPoints = 10;                                    // Amount of sensor readings to be read
        uint8_t writeIndex = 0;                                     // The index of the current writing position
        T sensorReadings[50];                                       // Stores last 50 sensor readings
        T maxReading = 0;                                           // Maximum sensor reading
        T minReading = 150000;                                      // Minimum sensor reading

    protected:
        /**
         * Appends new reading or overwrites existing reading to sensorReadings 
         * array. Stores data with FIFO-method. Fills up entire array before
         * wrapping around to the beginning. Also keeps track of the minimum and 
         * maximum readings
         * 
         * @param newReading new sensor reading to be appended
        */
        void addDataPoint(T newReading) {
            sensorReadings[writeIndex] = newReading;                // Write to array

            writeIndex++;                                           // Advance to the next position in the array
            if (writeIndex >= 50) writeIndex = 0;                   // If at the end of the array wrap around to the beginning

            if (newReading > maxReading) maxReading = newReading;   // Update the maxReading if new maximum is reached
            if (newReading < minReading) minReading = newReading;   // Update the minReading if new minimum is reached
        }

    public:
        /**
         * Calculates running average of a user defined number of data points by 
         * summing up the last "dataPoints"-amount of readings 
         * 
         * @returns running average
        */
        T getAverage() {
            T sum = 0;
            for (uint8_t pointsBack = 1; pointsBack <= dataPoints; pointsBack++) { 
                int8_t readIndex = writeIndex - pointsBack;         // Iterates back through data points
                if (readIndex < 0) readIndex += 50;                 // If at the beginning of the array wrap around to the end

                sum += sensorReadings[readIndex];                   // Sums up data
            }
            return sum / dataPoints;
        }

        /**
         * @returns maxReading the maximum sensor reading since reset
         */
        int getMax() {    
            return maxReading;
        }

        /**
         * @returns minReading the minimum sensor reading since reset
         */
        int getMin() {
            return minReading;
        }

        /**
         * Resets all readings in sensorReadings to 0
        */
        void resetSensorReadings() {
            for (uint8_t i = 0; i < 50; i++) sensorReadings[i] = 0;
        }

        /**
         * Resets maximum and minimum readings to initial values
         */
        void resetMaxAndMin() {
            maxReading = 0;
            minReading = 150000;
        }

        /**
         * Sets number of data points getAverage() should use when calculating
         * the running average
         * 
         * @param value the number of data points
        */
        void setDataPoints(uint8_t value) {
            dataPoints = value;
        }
};

/**
 * 
 */
class AlarmSystem {
    private:
        int alarmLED1;                                              // Declare variables for use in class
        int alarmLED2;
        int alarmBuzzer;
        bool toggleAlarmState;
        bool nextServoState;
        bool servoState;

    public:
        /**
         * 
         */
        AlarmSystem(int Led1Pin, int Led2Pin, int buzzerPin) {      // Two red alarm LEDs should blink alternately, while alarm sound from buzzer
            pinMode(Led1Pin, OUTPUT);
            pinMode(Led2Pin, OUTPUT);

            alarmLED1 = Led1Pin;
            alarmLED2 = Led2Pin;
            alarmBuzzer = buzzerPin;
        }

        /**
         * 
         */
        void alarm() {

            if (toggleAlarmState) {                                     // State machine to toggle between LEDs and buzzer pitch
                digitalWrite(alarmLED1, HIGH);
                digitalWrite(alarmLED2, LOW);
                tone(alarmBuzzer, 800);                 
                toggleAlarmState = !toggleAlarmState;
            }

            else if (!toggleAlarmState) {
                digitalWrite(alarmLED1, LOW);
                digitalWrite(alarmLED2, HIGH);
                tone(alarmBuzzer, 1000);                    
                toggleAlarmState = !toggleAlarmState;
            }
        }

        /**
         * 
         */
        void resetAlarm() {
            digitalWrite(alarmLED1, LOW);                               // Set LEDs low and stops buzzer                    
            digitalWrite(alarmLED2, LOW);
            noTone(alarmBuzzer);                                         
        }
        
        /**
         * 
        */
        void setNextServoState(bool value) {
            nextServoState = value;
        }

        /**
         * 
        */
        bool getNextServoState() {
            return nextServoState;
        }

        /**
         * 
        */
        void servoOn() {
          servoState = true;
        }

        /**
         * 
        */
        void servoOff() {
          servoState = false;
        }

        /**
         * 
        */
        bool isServoOn() {
          return servoState;
        }
};

/**
 * Configures HCSR04 sensor when object is constructed. Contains method for reading
 * distance in centimeters. Needs to be instantiated with pin numbers as arguments.
 * Derived from SensorData with type int
*/
class HCSR04UltrasonicSensor : public SensorData<int> {
    private:
        uint8_t trigger;
        uint8_t echo;

    public:
        /**
         * Constructs object with pin numbers and configures pins
         * 
         * @param trigPin the output to sensor
         * @param echopin the input from sensor
        */
        HCSR04UltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
            pinMode(trigPin, OUTPUT);
            pinMode(echoPin, INPUT);
            
            trigger = trigPin;
            echo = echoPin;
        }

        /**
         * Sends pulse and waits for it to return. Measures the duration and 
         * calculates the distance to the object
         * 
         * @returns distance the distance to the object in centimeters
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
 * Configures TMP36 sensor when object is constructed. Contains method for reading
 * temperature in celcius. Needs to be instantiated with pin number as argument.
 * Derived from SensorData with type float
*/
class TMP36TemperatureSensor : public SensorData<float> {
    private:
        uint8_t inputPin;

    public:
        /**
         * Constructs object with pin number and configures pin
         * 
         * @param pinNumber the input from sensor
        */
        TMP36TemperatureSensor(uint8_t pinNumber) {
            pinMode(pinNumber, INPUT);
            analogSetPinAttenuation(pinNumber, ADC_6db);            //Sets pin attenuation from 11dB (default) to 6 dB
            inputPin = pinNumber;  
        }

        /**
         * Calculates temperature based on input
         * 
         * @returns temperature the calculated temperature in celsius
        */
        float getTemperature() {
            float voltage = analogRead(inputPin) * 2000.0 / 4095.0;
            float temperature = (voltage - 500.0) / 10.0;

            addDataPoint(temperature);
            return temperature;
        }
};

/**
/**
 * Contains method for reading VL6180X range sensor in millimeters. Needs an instance
 * of Adafruit_VL6180X. Derived from SensorData with type float. Class serves only to
 * maintain consistensy in how readings from all sensors are obtained
*/
class VL6180XRangeSensor : public SensorData<int> {
    private:
        Adafruit_VL6180X* vlPointer;

    public:
        /**
         * @param instancePointer a pointer to an instance of Adafruit_VL6180X
        */
        VL6180XRangeSensor(Adafruit_VL6180X* instancePointer) {
            vlPointer = instancePointer;
        }
        /**
         * Receives range from I2C if sensor is available
         * 
         * @returns distance the distance to the object in millimeters
        */
        uint8_t getDistance() {
            uint8_t distance;
            if (vlPointer->begin()) distance = vlPointer->readRange();  //Accessing members of instance without manually deferencing pointer

            addDataPoint(distance);
            return distance;
        }
};

/**
 * Contains method for reading VL6180X lux sensor in lux. Needs an instance of 
 * Adafruit_VL6180X. Derived from SensorData with type float. Class serves only to
 * maintain consistensy in how readings from all sensors are obtained
*/
class VL6180XLuxSensor : public SensorData<float> {
    private:
        Adafruit_VL6180X* vlPointer;

    public:
        /**
         * @param instancePointer a pointer to an instance of Adafruit_VL6180X
        */
        VL6180XLuxSensor(Adafruit_VL6180X* instancePointer) {
            vlPointer = instancePointer;
        }

        /**
         * Receives lux from I2C if sensor is available
         * 
         * @returns lux the lux measured in lux
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
AlarmSystem alarmSystem(18, 19, 17);
WidgetTerminal terminal(V17);
WidgetLED BlynkAlarmLED(V20);

/**
 * Slider in Blynk app that sets the number of data points the running average 
 * should contain between 2 and 50 points
*/
BLYNK_WRITE(V0) {
    uint8_t dataPoints = param.asInt();

    temp.setDataPoints(dataPoints); 
    dist.setDataPoints(dataPoints);
    vlDist.setDataPoints(dataPoints); 
    vlLux.setDataPoints(dataPoints);
}

/**
 * 
*/
BLYNK_WRITE(V17) {
    if (String("help") == param.asStr()) {
        terminal.clear();
        terminal.println("Terminal commands:"); terminal.println("-----------------------------");
        terminal.println("\"values\" - Get readings from sensors");
        terminal.println("\"clear\"  - Clear terminal");
    }

    else if (String("values") == param.asStr()) {                                                             // Print sensor data to terminal
        terminal.print("Temperature:      "); terminal.print(temp.getTemperature()); terminal.println(" °C"); // Print data for every sensor
        terminal.print("Dist. ultrasonic: "); terminal.print(dist.getDistance());    terminal.println(" cm");
        terminal.print("Dist. ToF:        "); terminal.print(vlDist.getDistance());  terminal.println(" mm");
        terminal.print("Lux:              "); terminal.print(vlLux.getLux());        terminal.println(" lux");
    }

    else if (String("clear") == param.asStr()) {                    // Clear terminal
        terminal.clear();
        terminal.println("Done.");
    }

    else {
        terminal.println("The command you provided does not exist.");
    }

    terminal.flush();                                               // Ensure that all is sent to Blynk
}

/**
 * Slider in Blynk app that sets the position of servo between 0 and 180 degrees
*/
BLYNK_WRITE(V18) {
    servo.write(param.asInt());
}

//FOR TESTING PURPOSES ONLY
BLYNK_WRITE(V21) {
    vbuttonState = param.asInt();
}


/**
 * Sends real time and average readings to Blynk server every 1 second.
 * Widget's respective reading frequency should be set to PUSH in Blynk app. 
 * Should not send more than 10 values per second
*/
void sendReadings() {
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
 * Sends and resets maximum and average readings to Blynk server every 30 
 * seconds. Widget's respective reading frequency should be set to PUSH in 
 * Blynk app. Should not send more than 10 values per second
 */
void sendMaxAndMin() {
    Blynk.virtualWrite(V9, temp.getMax());         
    Blynk.virtualWrite(V10, temp.getMin());
    Blynk.virtualWrite(V11, dist.getMax());
    Blynk.virtualWrite(V12, dist.getMin());
    Blynk.virtualWrite(V13, vlDist.getMax());
    Blynk.virtualWrite(V14, vlDist.getMin());
    Blynk.virtualWrite(V15, vlLux.getMax());
    Blynk.virtualWrite(V16, vlLux.getMin());

    temp.resetMaxAndMin();
    dist.resetMaxAndMin();
    vlDist.resetMaxAndMin();
    vlLux.resetMaxAndMin();
}

/**
 * Checks if maximum or minimum readings is in accordance with thresholds
 * 
 * @returns true if two or more sensors have readings not in accordance 
 *      with thresholds, false otherwise
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
void toggleAlarm() {

    static bool resetAlarmOnce;                                     // Interlock to keep alarmsystem from resetting every 500 ms when no alarm
    static bool alarmLEDOnOnce = true;                              // Interlck to prevent weird sensor readings due to continuous BlynkAlarmLED.on() call on alarm    

    if (vbuttonState) { //ONLY FOR TESTING. REMOVE LATER AND UNCOMMET NEXT LINE.
    //if (alarmTrigger()) {                                           // Will go high as soon as two or more alarm levels is reached
        alarmSystem.alarm();                                        // Activate alarm system
        alarmSystem.servoOn();
        alarmSystem.setNextServoState(true);

        if (alarmLEDOnOnce) {
            BlynkAlarmLED.on();
            alarmLEDOnOnce = false;
        }
        resetAlarmOnce = true;
    } else if (resetAlarmOnce) { //ONLY FOR TESTING. REMOVE LATER AND UNCOMMENT NEXT LINE.
    //else if (resetAlarmOnce) {                        // Will reset alarm when false, but may be a bit slow, due to sensor values new every 30 s
        alarmSystem.setNextServoState(false);
        alarmSystem.resetAlarm();                                   // Reset alarm system
        BlynkAlarmLED.off();
        resetAlarmOnce = false;
        alarmLEDOnOnce = true;
    }
}

/**
 * Configures servo test if test button is activated. Stores state of button
 * each time button state changes
*/
BLYNK_WRITE(V19) {                                                   
    alarmSystem.servoOn();
    alarmSystem.setNextServoState(param.asInt());
}

/**
 * Timer that sweeps servo from between 0 and 180 degrees if test button is 
 * activated or if alarm is active. Should not have an interval less than 1
 * second because of indecisive servo position
*/
void sweepServo() {
    static bool endPosition = true;                                 // State machine that alters end position of servo
    static bool position = true;                                    // State machine that alters position of servo
    
    if (alarmSystem.isServoOn()) {                                     // Servo condition. Test or alarm activated
        if (!alarmSystem.getNextServoState()) {                        // End condition. Test button off
            position = endPosition;
            endPosition = !endPosition;
            alarmSystem.servoOff();// Resets to default configuration
        }

        if (position) servo.write(180);
        else servo.write(0);

        position = !position;
    }    
} 

/**
 * Resets data point slider to 10. Called each time Blynk app starts. Requires
 * that notify is enabled in Blynk app
*/
BLYNK_APP_CONNECTED() {
    Blynk.virtualWrite(V0, 10);                                     
}

/**
 * Sends formated HTML code showing sensor readings to web server. Buffer should
 * be increased HTML code is increased. Called each time server.handleClient() 
 * registers an event on "IP"/
*/
void handleRoot() {
    char buffer[600];
    float temperature = temp.getAverage();
    float lux = vlLux.getAverage();
    int distance1 = vlDist.getAverage();
    int distance2 = dist.getAverage();

    snprintf(buffer, 600, HTML,                                     // Formats sensor data into HTML code
             distance2, 
             temperature,
             distance1,
             lux);

    server.send(200, "text/html", buffer);                          // HTTP code 200, display HTML code
}

/**
 * Does initial configuration that is not handled by class constructors
*/
void setup() {
    Serial.begin(115200);                                           // Debug console
    Blynk.begin(AUTH, SSID, PASS, IPAddress(91,192,221,40), 8080);
    server.begin();                                                 // Initiates web server
    server.on("/", handleRoot);                                     // Manage HTTP request and run handleRoot() when "IP"/ is searched in browser
    Serial.println("HTTP server started");

    terminal.clear(); 
    terminal.println("Blynk program running");
    terminal.println("Type \"help\" for commands");
    terminal.flush();

    servo.setPeriodHertz(50);                                       // Standard 50hz servo
    servo.attach(16, 500, 2400);                                    // Servo connected to GPIO16

    timer.setInterval(500L, toggleAlarm);                           // Timer that should toggle alarm LED state and buzzer pitch when alarm
    timer.setInterval(1000L, sweepServo);                           // Timer that sweeps servo from outer to outer position with 1 s interval
    timer.setInterval(1000L, sendReadings);                         // Setup a function to be called every second and minute
    timer.setInterval(30000L, sendMaxAndMin);                       // Timer that pushes max/min values every 30 s interval

    dist.resetSensorReadings();                                     // Initialize all the readings to 0
    temp.resetSensorReadings();
    vlDist.resetSensorReadings();
    vlLux.resetSensorReadings();
}

/**
 * Calles methods that requires constant update. Should be used sparingly
 * because WiFi interruptions causes ESP32 to throw exception
*/
void loop() {
    Blynk.run();
    timer.run();                                                    // Initiates BlynkTimer
    server.handleClient();                                          // Checking web server to manage occurring events
}
