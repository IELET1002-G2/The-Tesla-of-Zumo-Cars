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
        const int threshold = 200;                             //Threshold for line sensors


        void rotate(int degrees)
        {
            float arcCounts = 7.63125*degrees;                  //Calibrated number of counts for given angle

            int leftStart = encoders.getCountsLeft();           //Start counts left
            int rightStart = encoders.getCountsRight();         //Start counts right

            int sum = 0;

            motors.setSpeeds(0, 0);                             //Stop motor before rotation
            delay(50);                                          //Delay to get rid of momentum

            while (encoders.getCountsRight() - rightStart < arcCounts) 
            {
                int left = encoders.getCountsLeft() - leftStart;        //Left counts since start
                int right = encoders.getCountsRight() - rightStart;     //Right counts since start
                int err = left + right;                                 //Error based on difference in counts

                sum += err;                                             //Integral of error
                
                int adjust = 0.1*err + 0.001*sum;                       //Calculates weighted adjustment

                left = constrain(-200 + adjust, -400, 400);             //New left speed
                right = constrain(200 - adjust, -400, 400);             //New right speed
                
                motors.setSpeeds(left, right);                          //Set motor speeds
            }

            motors.setSpeeds(0, 0);                                     //Stop motors after rotation
            delay(50);                                                  //Delay to get rid of momentum
        }


        int encoderPD(int leftStart, int rightStart, int last)
        {
            int left = encoders.getCountsLeft() - leftStart;        //Left counts since start
            int right = encoders.getCountsRight() - rightStart;     //Right counts since start
            int err = left - right;                                 //Error based on difference in counts
            
            int adjust = 0.5*err + 0.1*(err - last);                //Calculates weighted adjustment

            left = constrain(200 - adjust, -400, 400);              //New left speed
            right = constrain(200 + adjust, -400, 400);             //New right speed
            
            motors.setSpeeds(left, right);                          //Set motor speeds

            return err;                                             //Return error for next deriavative
        }


        void line(unsigned long time) 
        {
            unsigned long start = millis();                             //Store start time

            int leftCounter = encoders.getCountsLeft();                 //Start counts left
            int rightCounter = encoders.getCountsRight();               //Start counts right

            while (millis() - start < time)                             //Drives forward for set amount of time
            {
                static int last = 0;
                last = encoderPD(leftCounter, rightCounter, last);      //Adjusts motor speeds and stores return value
            }
            motors.setSpeeds(0,0);                                      //Stops motors at end of time period
            delay(50);                                                  //Delay to get rid of momentum
        }


        int PD(int input, int last, int speed, int batteryLevel = 100, bool emergencyPower = false)
        {
            int error = 2000 - input;                                   //Converts position to error based on desired position
            float batteryCorr = 1.00E+00 - exp(-1.00E-01*batteryLevel); //Correction for battey level

            int adjust = 0.4*error + 2.0*(error-last);                  //Adjustment based on error and deriavative

            int left = constrain(speed - adjust, -400, 400);            //Left motor speed based on adjustment
            int right = constrain(speed + adjust, -400, 400);           //Right motor speed based on adjustment

            if (batteryLevel <= 10 && !emergencyPower) {                //If battery level is to low, stop motors
                left = 0;                          
                right = 0;
            }

            motors.setSpeeds(left*batteryCorr, right*batteryCorr);      //Set speeds adjusted for battery level

            return error;                                               //Return error for next deriavative
        }


    public:

        bool noLine()
        {
            if (
                lineSensorValues[0] < threshold &&          //While each value is under threshold, return true
                lineSensorValues[1] < threshold &&          //If line is found, return false
                lineSensorValues[2] < threshold &&
                lineSensorValues[3] < threshold &&
                lineSensorValues[4] < threshold
            ) {
                return true;
            }
            else return false;

        }
        
        
        void noLineFound()                                  //Car has 2 seconds to find, and find back to, line if no line is detected
        {
            const unsigned long timeThreshold = 2000;       //Time interval before car should rotate 180 deg and go back
            static unsigned long timeStart = millis();      //Time since no line was found

            if (!noLine()) timeStart = millis();            //If line is found, update timeStart so car does not turn around

            if (millis() - timeStart > timeThreshold)       //If car has not found line in 2 seconds, rotate 180 deg and drive straight back
            {
                rotate(180);
                motors.setSpeeds(200, 200);
                timeStart = millis();                       //Update timeStart so car does not turn around again
            }

            /*
            if (noLine() && (millis() - timeStart) < timeThreshold))    //If each value is under threshold and time interval has not passed, continue driving
            {                                                           //If the line was found within 2 seconds, the car follows line as normal
                return;             
            }

            if ((millis() - timeStart) > timeThreshold)
            {
                rotate(180);                                //Rotates to go back to intersection
                motors.setSpeeds(200, 200);
            }

            while ((millis() - timeStart) > timeThreshold)
            {
                if (!noLine()) {
                    break;
                }
            }
            */
        }


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


        void followLinePD(int speed, int batteryLevel)
        {
            static int last = 0;
            int position = lineSensors.readLine(lineSensorValues);  //Reads position from lineSensors
            last = PD(position, last, speed, batteryLevel);         //Adjusts motors based on position and stores return value
        }


        void square()
        {
            for (byte n = 0; n < 4; n++) {
                line(3000);                             //Drive forward
                rotate(90);                             //Turn 90 degrees
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
            line(3000);                             //Drive forward
            rotate(180);                            //Turn 180 degrees
            line(3000);                             //Drive back to origin
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


        void print(char* string)
        {
            static unsigned long timer = millis();              //Variabel som lagrer tida for siste print

            if (millis() - timer > 100) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
                lcd.clear();
                lcd.print(string);                               //Printer tal utan desimalar
                lcd.gotoXY(0, 1);
                lcd.print("<A B^ C>");
                
                timer = millis();                               //Lagrer tida
            }
        }


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

            if (usbPowerPresent() && (buttonA.isPressed() || buttonC.isPressed() || force)) {   //Prompts configuration in serial monitor.
                if (!Serial) Serial.begin(9600);                                                //Initiates serial monitor.
                while (!Serial);                                                                //Waits for serial communication.

                lcd.clear();

                Serial.print(                                                                   //Prints selection of modes on monitor.
                    "Select mode:\n\n"
                    "q: Quit serial communication\n"
                    "0: Calibrate linesensors\n"
                    "1: Line follower\n"
                    "2: Object follower\n"
                    "3: Square\n"
                    "4: Circle\n"
                    "5: Back and forth\n"
                    "6: Slalom\n\n"
                    );
                
                while (Serial) {
                    if (Serial.available()) {                                           //If received bytes larger than zero.
                        config[0] = Serial.parseInt();                                  //Reads integer.

                        switch (config[0]) {
                            case 'q':
                                Serial.print("Quiting...\n\n");
                                Serial.end();                                           //Ends serial communication.
                                break;

                            case 1:                                                     //Prompts line follower configuration.
                                Serial.print(                                           //Prints selection of configurations.
                                    "Configure mode:\n\n"
                                    "   0: Normal\n"
                                    "   1: PD-regulated\n\n"
                                    "Selected configuration: "
                                    );
                                break;

                            case 6:                                                     //Prompts slalom configuration.
                                Serial.print("Enter centimeters between cones: ");      //Asks for distance.
                                break;
                        }

                        while (Serial && (config[0] == 1 || config[0] == 6)) {          //If mode need configuration.
                            if (Serial.available()) {
                                config[1] = Serial.parseInt();                          //Stores configuration.

                                Serial.print(config[1]);                                //Prints configuration to monitor.
                                break;
                            }
                        }
                        
                        Serial.print(
                            "\n\nConfiguration done.\n\n"
                            "Press button B to activate.\n\n"
                            );       
                        lcd.print("Ready.");

                        buttonB.waitForRelease();   //Waits for button B before initializing configuration.
                        lcd.clear();
                        break;        
                    }
                }
            }

            if (buttonB.getSingleDebouncedRelease()) {                          //Pauses if button B is pressed.
                motors.setSpeeds(0,0);
                lcd.clear();
                lcd.print("B to continue");
                lcd.gotoXY(0, 1);
                lcd.print("A or C to configure");
                
                while (true) {                                                  //Continuously checks if somthing happens.
                    if (buttonB.getSingleDebouncedRelease()) return config;     //Continues if button B is pressed again.
                    if (buttonA.isPressed() || buttonC.isPressed()){
                        enableForceConfig();                                    //Any other button prompts selection of modes.
                        break;  
                    }
                } 
            }
            
            if (buttonA.getSingleDebouncedRelease() || buttonC.getSingleDebouncedRelease() || force) {  //Prompts selection of modes.
                config[1] = 0;                                                                          //Resets configuration.

                while (true) {                                                          //Continuously checks if somthing happens.                                                     
                    print(modes[config[0]]);                                            //Prints current mode.

                    if (buttonA.getSingleDebouncedRelease()) config[0]--;               //Toggles back to previous mode.
                    if (buttonC.getSingleDebouncedRelease()) config[0]++;               //Toggles forward to following mode.
                    if (buttonB.getSingleDebouncedRelease()) {                          //Chooses current mode.
                        while (config[0] == 1) {                                        //Prompts line follower configuration.
                            if (config[1] == 0) print("Normal");                        //Prints current configuration.
                            if (config[1] == 1) print("PID");
                                        
                            if (buttonA.getSingleDebouncedRelease()) config[1]--;       //Toggles back to previous configuration.
                            if (buttonC.getSingleDebouncedRelease()) config[1]++;       //Toggles forward to following configuration.
                            if (buttonB.getSingleDebouncedRelease()) break;             //Chooses current configuration.

                            if (config[1] < 0) config[1] = 1;                           //Toggles to rightmost configuration.
                            if (config[1] > 1) config[1] = 0;                           //Toggles to leftmost configuration.
                        }

                        while (config[0] == 6) {                                        //Prompts slalom configuration.
                            print("cm: ");
                            print(config[1], 4, 0);                                     //Prints current distance between cones.

                            if (buttonA.getSingleDebouncedRelease()) config[1] -= 10;   //Increments distance between cones.
                            if (buttonC.getSingleDebouncedRelease()) config[1] += 10;   //Decrements distance between cones.
                            if (buttonB.getSingleDebouncedRelease()) break;             //Chooses current distance.
                                
                            if (config[1] < 0) config[1] = 100;                         //Toggles to rightmost configuration.
                            if (config[1] > 100) config[1] = 0;                         //Toggles to leftmost configuration.
                        }

                        break;                                                          //Leaves selection of modes.
                    }

                    if (config[0] < 0) config[0] = 6;                                   //Toggles to rightmost mode.
                    if (config[0] > 6) config[0] = 0;                                   //Toggles to leftmost mode.
                }
            }
            
            if (force) force = false;               //Turns of forcing.
            return config;                          //Returns pointer to array that stores configuration.
        }


        void print(int value, int X, int Y)
        {
            static unsigned long timer = millis();              //Variabel som lagrer tida for siste print

            if (millis() - timer > 100) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
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
        float level = 100.0;
        bool empty = false;


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


        int getBatteryLevel(float trip, float weight=0)
        {
            static float lastTrip = 0.0;

            level -= (trip-lastTrip)*(275+weight)/275;  //Rekner ut batterinivå
            lastTrip = trip;                                                      //Lagrer siste trip

            if (level <= 10) {                                                    //Viss batterinivået er under 10%
                empty = true;                                                     //Inkrementerer teljar
                ledRed(HIGH);
            }

            return constrain(level, 0, 100);                                      //Returnerer batterinivå
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




void setup() 
{}



void loop()
{
    float distance;
    int batteryLevel;

    int* config = intf.command();

    switch (config[0]) {
        case 0:
            drive.calibrateSensors();                                   //Kalibrerer sensorane på kommando
            intf.enableForceConfig();
            break;

        case 1:
            distance = motion.getTrip();                                //Henter distanse(tur) kjørt
            batteryLevel = battery.getBatteryLevel(distance);           //Henter batterinivå basert på distanse kjørt

            if (config[1] == 0) drive.followLine(batteryLevel);         //Korrigerer retning basert på posisjon
            else drive.followLinePD(300, batteryLevel);

            drive.noLineFound();

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