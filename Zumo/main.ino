#include <Zumo32U4.h>               //Importerer Zumo-biblioteket

Zumo32U4Motors motors;              //Oppretter instans av motorane
Zumo32U4Encoders encoders;          //Oppretter instans av kodarane
Zumo32U4LineSensors lineSensors;    //Oppretter instans av linjesensorane
Zumo32U4ButtonA buttonA;            //Oppretter instans av knapp A
Zumo32U4LCD lcd;                    //Oppretter instans av LCD-display
Zumo32U4Buzzer buzzer;              //Oppretter instans av buzzeren

unsigned int lineSensorValues[5];   //Verdien til kvar linjesensor



class SelfDriving
{
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


        void followLine(int value, int batteryLevel, bool emergencyPower = false, bool fastMode = false)
        {
            int leftSpeed;
            int rightSpeed;

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
};



class Interface
{
    public:

        void activate(char line11[], char line12[], char line21[], char line22[])
        {
            lcd.clear();
            lcd.print(line11);                              //Skriver første beskjed til første linje
            lcd.gotoXY(0, 1);
            lcd.print(line12);                              //Skriver første beskjed til andre linje

            buttonA.waitForButton();                        //Venter til knapp A blir trykka inn
            
            lcd.clear();
            lcd.print(line11);                              //Skriver andre beskjed til første linje
            lcd.gotoXY(0, 1);
            lcd.print(line12);                              //Skriver andre beskjed til andre linje
            
            delay(2000);                                    //Gir tid til å lese instruksjon

            lcd.clear();
        }


        void pause()
        {
            if (buttonA.isPressed()) {                      //Viss knapp A blir trykka inn
                motors.setSpeeds(0, 0);                     //Stopper motorane  
                
                lcd.clear();                                //Ventetid mellom knappetrykk
                lcd.clear();
                lcd.print("Press A");
                lcd.gotoXY(0, 1);
                lcd.print("to start");
                
                delay(2000);                                //Gir tid til å slippe knappen
                buttonA.waitForButton();                    //Venter til knapp A blir trykka inn
                delay(2000);                                //Gir tid til å slippe bilen

                lcd.clear();
            }
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
                lcd.gotoXY(X, Y);                               //Går til valgt posisjon på LCD
                lcd.print(value);                               //Printer tal utan desimalar
                
                timer = millis();                               //Lagrer tida
            }
        }


        void writeTwoLines(char firstLine[], char secondLine[])
        {
            lcd.clear();                                        //Nullstiller LCD
            lcd.print(firstLine);                               //Skriver første linje
            lcd.gotoXY(0, 1);                                   //Går til første segment på nedste rad
            lcd.print(secondLine);                              //Skriver andre linje
        }
};



class Motion
{
    private:

        float speed;
        float trip;
        float distance;
        float displacement;


        void calculateMotion()
        {
            static unsigned long timer;                                 //Timer for å lagre tidsintervallet

            int leftCount = encoders.getCountsAndResetLeft();           //Leser av teljarane på venstre kodar
            int rightCount = encoders.getCountsAndResetRight();         //Leser av teljarane på høgre kodar

            float avgDisp = (leftCount + rightCount)/(2.0*7765.0);      //Gjennomsnittlig forflytning bilen har gått
            float avgDist = abs(avgDisp);                               //Distansen er absoluttverdien av forflytninga

            trip += avgDist;                                            //Akkumulerer trip-teljar
            distance += avgDist;                                        //Akkumulerer distanse
            displacement += avgDisp;                                    //Akkumulerer forflytning

            speed = avgDisp/(millis() - timer)*1000;                    //Gjennomsnittsfarta
            timer = millis(); 
        }


    public:

        float getSpeed()
        {
            calculateMotion();          //Kalkulerer bevegelsen
            return speed;               //Henter gjennomsnittsfart
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

        float batteryLvl;
        byte emptyCounter;                                      //Antall gongar batteriet er under 10%


    public:

        void chargeBattery()
        {
            static unsigned long timer = millis();              //Variabel som lagrer tida for siste print
            static bool ledState = HIGH;                        //Variabel som lagrer tilstand til LED
            /*
            if (millis() - timer > 400) {                       //Blinker kvart 400ms
                ledRed(ledState);

                ledState = !ledState;                           //Toggler tilstand
            }
            */
            for (byte i = 0; i <= 100; i++) {
                batteryLvl = i;

                ledRed(ledState);
                ledState = !ledState;                           //Toggler tilstand
                delay(400);
            }
        }


        int getBatteryLevel(float trip, float weight = 0.0)
        {
            static float lastDistance = 0;

            batteryLvl -= (trip-lastDistance)*((275.0+weight)/275.0);
            if (batteryLvl < 0.0) batteryLvl = 0.0;

            lastDistance = trip;

            if (batteryLvl <= 10) {                             //Viss batterinivået er under 10%
                emptyCounter += 1;                              //Inkrementerer teljar
                ledRed(HIGH);
            }

            return batteryLvl;
        }


        bool getEmergencyPower()
        {
            return (emptyCounter == 1) ? true : false;          //Får resterande batteri første gong den er under 10%
        }
};



SelfDriving drive;                  //Instans for sjølvkjøring
Interface intf;                     //Instans for brukargrensesnitt
Motion motion;                      //Instans for henting av distansedata
Battery battery;                    //Instans for batteri



void setup()
{
    intf.activate("Press A", "to cal.", "Wait for", "cal.");    //Instruerer brukar om calibrering og venter på kommando
    drive.calibrateSensors();                                   //Kalibrerer sensorane på kommando
    intf.activate("Press A", "to start", "Press A", "to stop"); //Instruerer brukar om start og venter på kommando
}



void loop()
{
    int distance = motion.getTrip();                            //Henter distanse(tur) kjørt
    int batteryLevel = battery.getBatteryLevel(distance);       //Henter batterinivå basert på distanse kjørt
    int position = lineSensors.readLine(lineSensorValues);      //Leser av posisjonen til zumoen 

    drive.followLine(position, batteryLevel);                   //Korrigerer retning basert på posisjon

    intf.pause();                                               //Pause programmet ved å trykke A
    intf.print(distance, 0, 0);                                 //Printer posisjon til første linje på LCD
    intf.print(batteryLevel, 0, 1);                             //Printer batterinivå til andre linje på LCD
}
