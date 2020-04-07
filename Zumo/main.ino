#include <Zumo32U4.h>               //Importerer Zumo-biblioteket

Zumo32U4Motors motors;              //Oppretter instans av motorane
Zumo32U4Encoders encoders;          //Oppretter instans av kodarane
Zumo32U4LineSensors lineSensors;    //Oppretter instans av linjesensorane
Zumo32U4ButtonA buttonA;            //Oppretter instans av knapp A
Zumo32U4ButtonB buttonB;            //Oppretter instans av knapp A
Zumo32U4ButtonC buttonC;            //Oppretter instans av knapp A
Zumo32U4LCD lcd;                    //Oppretter instans av LCD-display
Zumo32U4Buzzer buzzer;              //Oppretter instans av buzzeren



class SelfDriving
{
    private:

        unsigned int lineSensorValues[5];   //Verdien til kvar linjesensor

        int leftSpeed;
        int rightSpeed;


        void rotate(int degrees)
        {
            //Teoretisk count per meter er 909.7(2*pi*r)=7425

            int counts = encoders.getCountsLeft();
            float arcCounts = 0.267*7425*degrees/360;       //Buelengda på 360-rotasjon er 0.267m
            
            rightSpeed = 200*degrees/abs(degrees);          //Endrer forteiknet avhengig av forteikn på vinkel
            leftSpeed = -rightSpeed;

            motors.setSpeeds(0, 0);
            delay(50);

            while (abs(encoders.getCountsLeft() - counts) < abs(arcCounts)) {
                motors.setSpeeds(leftSpeed, rightSpeed);
            }

            motors.setSpeeds(0, 0);
            delay(50);

            //motors.setSpeeds(leftSpeed, rightSpeed);
            //delay(800*abs(degrees)/360);                         //Går ut ifrå 800ms ved 200 gir 360 graders rotasjon
        }


    public:

        void calibrateSensors() 
        {
            lineSensors.initFiveSensors();                  //Starter 5-linjesensorkonfigurasjonen

            for (int t = 0; t <= 200; t++) {                //Varer i 4000ms der t er tid i ms
                lineSensors.calibrate();                    //Kalibrerer sensor

                int speed = 400*sin(PI/100*t);              //Farten er beskrive av sinus-uttrykk med periode 4000ms
                
                motors.setSpeeds(speed, -speed);            //Bilen roterer med varierande fart
                delay(1);                                   //Delay for å styre tida
            }
        }


        void followLine(int batteryLevel, bool emergencyPower = false, bool fastMode = false)
        {
            int value = lineSensors.readLine(lineSensorValues);         //Leser av posisjonen til zumoen 
            float batteryCorr = 1.00E+00 - exp(-1.00E-01*batteryLevel); //Korreksjonsfaktor for batterinivå

            if (fastMode) {                                             //Rask modus
                leftSpeed = 4.00E+02 - 8.00E+02*exp(-2.50E-03*value);   //Farten er bestemt av eksponentialfunksjonar
                rightSpeed = 4.00E+02 - 3.63E-02*exp(+2.50E-03*value);
            }
            else {
                leftSpeed =                             //Venstre fart er lik eit 4.gradspolynom der variabelen 
                    -4.00E+02*pow(value, 0)             //er sensorverdien med Df = [0, 4000].
                    +1.04E+00*pow(value, 1)             //Polynomet er stigande for dette intervallet med 
                    -6.63E-04*pow(value, 2)             //verdiar Vf = [-400, 400].
                    +1.80E-07*pow(value, 3)             //Polynomet har terrassepunkt i (2000, 200).
                    -1.67E-11*pow(value, 4);

                rightSpeed =                            //Høgre fart er lik eit 4.gradspolynom der variabelen er
                    +4.00E+02*pow(value, 0)             //sensorverdien med Df = [0, 4000].
                    -1.07E-01*pow(value, 1)             //Polynomet er synkande for dette intervallet med 
                    -1.03E-04*pow(value, 2)             //verdiar Vf = [-400, 400].
                    +8.67E-08*pow(value, 3)             //Polynomet har terrassepunkt i (2000, 200).
                    -1.67E-11*pow(value, 4);
            }

            leftSpeed *= batteryCorr;                   //Korrigerer med batterinivået
            rightSpeed *= batteryCorr;

            if (batteryLevel <= 10 && !emergencyPower) {//Viss batterinivået er 10% og nødbatteri ikkje er aktivert
                leftSpeed = 0;                          //Setter fartane til null
                rightSpeed = 0;
            }
            
            motors.setSpeeds(leftSpeed, rightSpeed);    //Setter fart til utrekna, korrigerte verdiar
        }


        void square()
        {
            for (byte n = 0; n < 4; n++) {
                motors.setSpeeds(200, 200);
                delay(2000);
                rotate(90);
            }
        }


        void circle()
        {
            motors.setSpeeds(300, 150);
            delay(3000);
            motors.setSpeeds(0, 0);
        }


        void backAndForth()
        {
            motors.setSpeeds(200, 200);
            delay(2000);
           
            rotate(180);
            
            motors.setSpeeds(200, 200);
            delay(2000);
            motors.setSpeeds(0, 0);

        }


        void slalom()
        {
            motors.setSpeeds(0, 0);
            delay(50);
            
            int counts = encoders.getCountsLeft();
            float coneCounts = 0.5*7425;            //Kjegledistanse på 0.5m
            unsigned long time = millis();

            while (encoders.getCountsLeft() - counts < coneCounts) {
                motors.setSpeeds(200, 200);
            }

            time = millis() - time;
            
            for (byte i = 0; i < 10; i++){          //10 kjegler
                rotate(90);
                motors.setSpeeds(200, 200);
                delay(time);
                rotate(-90);
                motors.setSpeeds(200, 200);
                delay(time);
                rotate(-90);
                motors.setSpeeds(200, 200);
                delay(time);
                rotate(90);
            }
        }
};



class Interface
{
    private:

        bool force = true;      //Variable that controls whether or not buttons are required to prompt configuration.


    public:

        int* command()
        {
            static int config[] = {0, 0};                                       //Array stores mode and configuration.
            char* modes[] = {                                                   //Array with names of modes.
                "Calib",
                "Line",
                "Object",
                "Square",
                "Circle",
                "B and F",
                "Slalom"    
                };

            auto AorC_getSingleDebouncedRelease = [] {                         //Anonymous function that returns whether
                return                                                          //or not A or C have been pressed.
                    buttonA.getSingleDebouncedRelease() ||
                    buttonC.getSingleDebouncedRelease();
            };

            auto toggle = [](byte increment, byte max) {                        //Anonymous function that toggles.
                if (buttonA.getSingleDebouncedRelease()) config[0] -= increment;//Toggles back to previous mode.
                if (buttonC.getSingleDebouncedRelease()) config[0] += increment;//Toggles forward to following mode.
                if (config[1] < 0) config[1] = max;                             //Toggles to rightmost configuration.
                if (config[1] > max) config[1] = 0;                             //Toggles to leftmost configuration.
            };

            auto getSerial = [](byte configMenu) {                              //Anonymous function that reads serial.
                if (Serial && Serial.available()) {                             
                    config[configMenu] = Serial.parseInt();                     //Gets mode or configuration from monitor.
                    return true;                                                //Controls flow if configuration received.
                } 
            };
            
            if (buttonB.getSingleDebouncedRelease()) {                          //Pauses if button B is pressed.
                motors.setSpeeds(0, 0);
                print("B:cont", "A/C:conf");
                
                while (true) {                                                  //Continuously checks if somthing happens.
                    if (buttonB.getSingleDebouncedRelease()) return config;     //Continues if button B is pressed again.
                    if (AorC_getSingleDebouncedRelease()) {                    //Any other button prompts selection of modes.
                        enableForceConfig();
                        break;                                                  
                    }
                } 
            }

            if (AorC_getSingleDebouncedRelease() || force) {                   //Prompts selection of modes.
                config[1] = 0;                                                  //Resets configuration.
                
                if (usbPowerPresent() && !Serial) {
                    Serial.begin(9600);                                         //Initiates serial monitor.
                    while (!Serial);
                }
                if (Serial) {
                    Serial.print(                                               //Prints selection of modes on monitor.
                        "Select mode:\n\n"
                        "0: Calibrate linesensors\n"
                        "1: Line follower\n"
                        "2: Object follower\n"
                        "3: Square\n"
                        "4: Circle\n"
                        "5: Back and forth\n"
                        "6: Slalom\n\n"
                        );
                }

                while (true) {                                                          //Continuously checks if somthing happens.                                                     
                    print(modes[config[0]], "<A B^ C>");                                //Prints current mode.

                    toggle(1, 6);                                                       //Toggles mode.
                                                                         
                    if (buttonB.getSingleDebouncedRelease() || getSerial(0)) {                          //Chooses current mode.
                        while (config[0] == 1) {                                        //Prompts line follower configuration.
                            for (bool once = true; once && Serial; once = false) {      //Runs exactly once.
                                Serial.print(                                           //Prints selection of configurations.
                                    "Configure mode:\n\n"
                                    "   0: Normal\n"
                                    "   1: PD-regulated\n\n"
                                    "Selected configuration: "
                                    );
                            }

                            if (config[1] == 0) print("Normal", "<A B^ C>");            //Prints current configuration.
                            if (config[1] == 1) print("PID", "<A B^ C>");

                            toggle(1, 1);                                               //Toggles configuration.

                            if (buttonB.getSingleDebouncedRelease() || getSerial(1)) break; //Chooses current configuration.
                        }

                        while (config[0] == 6) {                                        //Prompts slalom configuration.
                            for (bool once = true; once && Serial; once = false) {      //Runs exactly once.
                                Serial.print("Enter centimeters between cones: ");      //Asks for distance.
                            }

                            print("cm: ", "<A B^ C>");
                            print(config[1], 4, 0);                                     //Prints current distance between cones.

                            toggle(10, 100);                                            //Toggles configuration.

                            if (buttonB.getSingleDebouncedRelease() || getSerial(1)) break; //Chooses current distance.
                        }

                        break;                                                          //Leaves selection of modes.
                    }
                }
            }

            if (Serial) {
                Serial.print(config[1]);
                Serial.print(
                    "\n\nConfiguration done.\n\n"
                    "Press button B to activate.\n\n"
                    );
            }
                   
            print("Ready.", "Press B");
            buttonB.waitForRelease();
            lcd.clear();
            
            force = false;                                      //Turns of forcing.
            return config;                                      //Returns pointer to array that stores configuration.
        }


        void print(int value, int X, int Y)
        {
            static unsigned long timer = millis();              //Variabel som lagrer tida for siste print

            if (millis() - timer > 100) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
                lcd.clear();
                lcd.gotoXY(X, Y);                               //Går til valgt posisjon på LCD
                lcd.print(value);                               //Printer tal med decimalar
                
                timer = millis();                               //Lagrer tida
            }
        }


        void print(float value, int X, int Y)
        {
            static unsigned long timer = millis();              //Variabel som lagrer tida for siste print

            if (millis() - timer > 100) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
                lcd.clear();
                lcd.gotoXY(X, Y);                               //Går til valgt posisjon på LCD
                lcd.print(value);                               //Printer tal utan desimalar
                
                timer = millis();                               //Lagrer tida
            }
        }


        void print(char* string1, char* string2)
        {
            static unsigned long timer = millis();              //Variabel som lagrer tida for siste print

            if (millis() - timer > 100) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
                lcd.clear();
                lcd.print(string1);                               //Printer tal utan desimalar
                lcd.gotoXY(0, 1);
                lcd.print(string2);
                
                timer = millis();                               //Lagrer tida
            }
        }


        void printMessage(char message[])
        {
            lcd.clear();

            for (byte x = 0, y = 0; x < constrain(sizeof(message)-1, 0, 16); x++)   //Itererer gjennom stringen
            {
                if (x >= 8) {                                   //Viss stringen er lenger enn første rad
                    y = 1;                                      //Starter på neste rad.
                    lcd.gotoXY(x-8, y);                         
                    lcd.print(message[x]);                      //Printer bokstav
                }
                else {
                    lcd.gotoXY(x, y);
                    lcd.print(message[x]);                      //Printer bokstav
                }
            }
        }  


        void enableForceConfig()
        {
            force = true;           //Forces to configure a single time after call.
        }       
};



class Motion
{
    private:

        float momSpeed;
        float avgSpeed;
        float trip;
        float distance;
        float displacement;


        void calculateMotion()
        {
            static unsigned long timer;                                 //Timer for å lagre tidsintervallet

            int leftCount = encoders.getCountsAndResetLeft();           //Leser av teljarane på venstre kodar
            int rightCount = encoders.getCountsAndResetRight();         //Leser av teljarane på høgre kodar

            //Teoretisk count per meter er 909.7(2*pi*r)=7425

            float avgDisp = (leftCount + rightCount)/(2.0*7765.0);      //Gjennomsnittlig forflytning bilen har gått
            float avgDist = abs(avgDisp);                               //Distansen er absoluttverdien av forflytninga

            trip += avgDist;                                            //Akkumulerer trip-teljar
            distance += avgDist;                                        //Akkumulerer distanse
            displacement += avgDisp;                                    //Akkumulerer forflytning

            momSpeed = avgDisp/(millis() - timer)*1000;                 //Momentanfarta
            timer = millis(); 
        }


        float getAverageSpeed()
        {
            static unsigned long timer1 = millis();
            static unsigned long timer2 = millis();
            static unsigned long timeOver70;
            static unsigned long counter;
            static float sumOfSpeeds;
            static float highestSpeed;
            float maxSpeed;

            if (momSpeed > highestSpeed) highestSpeed = momSpeed;
            if (momSpeed < 0.7*maxSpeed) timer2 = millis();

            timeOver70 += millis() - timer2;

            if (momSpeed < 0) {
                sumOfSpeeds += momSpeed;
                counter++;
                
                if (millis() - timer1 >= 6000) {
                    avgSpeed = sumOfSpeeds/counter;

                    sumOfSpeeds = 0;
                    counter = 0;
                    timer1 = millis();

                    return avgSpeed;
                }
            }
        }


    public:

        float getSpeed()
        {
            calculateMotion();          //Kalkulerer bevegelsen
            return momSpeed;               //Henter gjennomsnittsfart
        }


        float getDistance()
        {
            calculateMotion();          //Kalkulerer bevegelsen
            return distance;            //Henter distansen
        }


        float getDisplacement()
        {
            calculateMotion();          //Kalkulerer bevegelsen
            return displacement;        //Henter forflytninga
        } 
        

        float getTrip()
        {
            calculateMotion();          //Kalkulerer bevegelsen
            return trip;                //Henter trip-teljar
        }


        void setTrip(int value)
        {
            trip = value;               //Setter trip-verdi
        }
};



class Battery
{
    private:

        int health;
        int cycles;
        float level;
        float lastTrip;
        bool empty;


    public:

        void chargeBattery()
        {
            static bool ledState = HIGH;                        //Variabel som lagrer tilstand til LED
            
            for (byte i = 0; i <= 100; i++) {
                level = i;

                ledRed(ledState);
                ledState = !ledState;                           //Toggler tilstand
                delay(400);
            }
        }


        int getBatteryLevel(float trip, float weight=0, float speed=0)
        {
            level = constrain(level - (trip-lastTrip)*(275+weight)/275, 0, 100);  //Rekner ut batterinivå
            lastTrip = trip;                                                      //Lagrer siste trip

            if (level <= 10) {                                                    //Viss batterinivået er under 10%
                empty = true;                                                     //Inkrementerer teljar
                ledRed(HIGH);
            }

            return (int)level;                                                    //Returnerer batterinivå
        }
    

        bool getEmergencyPower()
        {
            return empty;                                       //Får resterande batteri første gong den er under 10%
        }


        void resetEmpty()
        {
            empty = false;
        }
};



SelfDriving drive;                  //Instans for sjølvkjøring
Interface intf;                     //Instans for brukargrensesnitt
Motion motion;                      //Instans for henting av distansedata
Battery battery;                    //Instans for batteri




void setup() {}



void loop()
{
    int* config = intf.command();

    switch (config[0]) {
        case 0:
            drive.calibrateSensors();                                   //Kalibrerer sensorane på kommando
            intf.enableForceConfig();
            break;

        case 1:
            int distance = motion.getTrip();                            //Henter distanse(tur) kjørt
            int batteryLevel = battery.getBatteryLevel(distance);       //Henter batterinivå basert på distanse kjørt

            drive.followLine(batteryLevel);                   //Korrigerer retning basert på posisjon

            intf.print(distance, 0, 0);                                 //Printer posisjon til første linje på LCD
            intf.print(batteryLevel, 0, 1);                             //Printer batterinivå til andre linje på LCD
            break;

        case 2:
            //Konfigurer objektfølgar
            break;
        
        case 3:
            drive.square();
            intf.enableForceConfig();
            break;

        case 4:
            drive.circle();
            intf.enableForceConfig();
            break;

        case 5:
            drive.backAndForth();
            intf.enableForceConfig();
            break;

        case 6:
            drive.slalom();
            intf.enableForceConfig();
            break;

        default:
            intf.enableForceConfig();
            break;
    }
}
