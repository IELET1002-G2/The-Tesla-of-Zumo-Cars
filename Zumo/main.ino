#include <Zumo32U4.h>               //Importerer Zumo-biblioteket

Zumo32U4Motors motors;              //Oppretter instans av motorane
Zumo32U4Encoders encoders;          //Oppretter instans av kodarane
Zumo32U4LineSensors lineSensors;    //Oppretter instans av linjesensorane
Zumo32U4ButtonA buttonA;            //Oppretter instans av knapp A
Zumo32U4ButtonB buttonB;            //Oppretter instans av knapp A
Zumo32U4ButtonC buttonC;            //Oppretter instans av knapp A
Zumo32U4LCD lcd;                    //Oppretter instans av LCD-display
Zumo32U4Buzzer buzzer;              //Oppretter instans av buzzeren

unsigned int lineSensorValues[5];   //Verdien til kvar linjesensor



class SelfDriving
{
    private:

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


        void followLine(int value, bool fastMode, int batteryLevel)
        {
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

        /*
            if (millis() - timer > 400) {                       //Blinker kvart 400ms
                ledRed(ledState);

                ledState = !ledState;                           //Toggler tilstand
            }
        */
       /*
       void command()
       {
           byte i;
           byte i[3];
           bool statement;

            while (statement) {
                if (buttonA.isPressed()) i[0]--; 
                if (buttonC.isPressed()) i[0]++;
                if (buttonB.isPressed()) {
                    while (statement) {
                        if (buttonA.isPressed()) i[1]--; 
                        if (buttonC.isPressed()) i[1]++;
                        if (buttonB.isPressed()) {
                    
                        }
                    }
                }
            }
       }
       */

       void command()
       {
            while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                //Vis instruksjonar
                //Delay for å vente på fleire knappar
            }

            if (buttonA.isPressed() && buttonB.isPressed() && buttonC.isPressed()) {
                buttonA.waitForRelease();
                buttonB.waitForRelease();
                buttonC.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (buttonA.isPressed() && buttonB.isPressed()) {
                buttonA.waitForRelease();
                buttonB.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (buttonA.isPressed() && buttonC.isPressed()) {
                buttonA.waitForRelease();
                buttonC.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (buttonB.isPressed() && buttonC.isPressed()) {
                buttonB.waitForRelease();
                buttonC.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (buttonA.isPressed()) {
                buttonA.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (buttonB.isPressed()) {
                buttonB.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (buttonC.isPressed()) {
                buttonC.waitForRelease();
                while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) {
                    //Vis instruksjonar
                }

                if (buttonA.isPressed()) {
               
                }
                if (buttonB.isPressed()) {

                }
                if (buttonC.isPressed()) {

                }
            }
            else if (usbPowerPresent()) {
                Serial.begin(9600);
                while (!Serial);
                //Vis instruksjonar

                if (Serial.available()) {
                    switch (Serial.read()) {
                        case 'A':
                            break;
                    
                        default:
                            break;
                    }
                }
            }
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
            lastTrip = trip;                                                                //Lagrer siste trip

            if (level <= 10) ledRed(HIGH);                 //Lyser raudt viss batteriet er under 10%

            return (int)level;                             //Returnerer batterinivå
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
    int batteryLevel = battery.getBatteryLevel(distance, 0);    //Henter batterinivå basert på distanse kjørt
    int position = lineSensors.readLine(lineSensorValues);      //Leser av posisjonen til zumoen 

    drive.followLine(position, true, batteryLevel);             //Korrigerer retning basert på posisjon

    intf.pause();                                               //Pause programmet ved å trykke A
    intf.print(distance, 0, 0);                                 //Printer posisjon til første linje på LCD
    intf.print(batteryLevel, 0, 1);                             //Printer batterinivå til andre linje på LCD
}
