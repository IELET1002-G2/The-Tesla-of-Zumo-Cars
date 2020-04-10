/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
//Include sensor- and servo-libraries here

const char* AUTH = "";
const char* SSID = "";
const char* PASS = "";
const char* HTML = 
    "<!DOCTYPE html>\
    <html>\
      <body>\
        <h1>Gruppe 2: Web server</h1>\
      </body>\
    </html>";


BlynkTimer timer;
WebServer server;   //Port is 80 as default



class Sensor {
    protected:

        byte pin;

    public:

        void setPin(byte pinNumber) {
            pin = pinNumber;
            pinMode(pinNumber, INPUT);
        }

        void read() {
            analogRead(pin);
        }
};


class TMP36TemperatureSensor: public Sensor {
    public:
    
        TMP36TemperatureSensor(byte pinNumber) {
            setPin(pinNumber);
        }

        //More functions related to this class
};


class HCSR04UltrasonicSensor: public Sensor {
    private:

        byte trigger;

    public:

        HCSR04UltrasonicSensor(byte echoPin, byte trigPin) {
            setPin(echoPin);

            trigger = trigPin;
            pinMode(trigPin, OUTPUT);
        }

        //More functions related to this class
};


class VL6180XTimeOfFlight {
    private:

    public:

        //More functions related to this class
};



// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void timerEvent1() {
    // You can send any value at any time.
    // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V5, millis() / 1000);
}


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


char* floatToString(float number, byte precision) {
    static char string[10];
    
    return dtostrf(number, 0, precision, string);
}


void handleRoot() {
    server.send(200, "text/html", HTML);                            //Code 200, display HTML code.
}


void setup() {
    Serial.begin(115200);                                           // Debug console

    Blynk.begin(AUTH, SSID, PASS, IPAddress(91,192,221,40), 8080);
    server.begin();                                                 //Initiates web server.
    server.on("/", handleRoot);                                     //Manage HTTP request and run handleRoot() when "IP"/ is searched in browser.
    Serial.println("HTTP server started");

    timer.setInterval(1000L, timerEvent1);                          // Setup a function to be called every second and minute
    timer.setInterval(60000L, timerEvent2);
}


void loop() {
    Blynk.run();
    timer.run();                                                    // Initiates BlynkTimer
    server.handleClient();                                          //Checking web server to manage occurring events
}
