#include <Zumo32U4.h>               //Importerer Zumo-biblioteket
#include <Wire.h>
#include <LSM303.h>

Zumo32U4Motors motors;              //Oppretter instans av motorane
Zumo32U4Encoders encoders;          //Oppretter instans av kodarane
Zumo32U4LineSensors lineSensors;    //Oppretter instans av linjesensorane
Zumo32U4ButtonA buttonA;            //Oppretter instans av knapp A
Zumo32U4ButtonB buttonB;            //Oppretter instans av knapp A
Zumo32U4ButtonC buttonC;            //Oppretter instans av knapp A
Zumo32U4LCD lcd;                    //Oppretter instans av LCD-display
Zumo32U4Buzzer buzzer;              //Oppretter instans av buzzeren
LSM303 compass;                     //Oppretter instans av akselerometer
L3G gyro;




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
            float arcCounts = 0.342*7425*degrees/360;       //Buelengda på 360-rotasjon er 0.267m
            
            rightSpeed = 200*degrees/abs(degrees);          //Endrer forteiknet avhengig av forteikn på vinkel
            leftSpeed = -rightSpeed;

            motors.setSpeeds(0, 0);
            delay(50);

            while (abs(encoders.getCountsLeft() - counts) < abs(arcCounts)) {
                motors.setSpeeds(leftSpeed, rightSpeed);
            }

            motors.setSpeeds(0, 0);
            delay(50);
        }

        void line(float error, float speed)
        {
            int adjust = speed*error/200.0;

            int leftSpeed = constrain((speed-adjust), -400, 400);
            int rightSpeed = constrain((speed+adjust), -400, 400);

            motors.setSpeeds(leftSpeed, rightSpeed);
        }

    public:

        void rot(int d) {rotate(d);}

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
                int startLeft = encoders.getCountsLeft();
                int startRight = encoders.getCountsRight();

                unsigned long timer = millis();
                while (millis() - timer < 1000)
                {
                    int diff = (encoders.getCountsLeft()-startLeft) - (encoders.getCountsRight()-startRight);
                    line((float)diff, 200);
                    delay(1);
                }
                motors.setSpeeds(0,0);
                delay(500);
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
            int startLeft = encoders.getCountsLeft();
            int startRight = encoders.getCountsRight();

            unsigned long timer = millis();
            while (millis() - timer < 2000)
            {
                int diff = (encoders.getCountsLeft()-startLeft) - (encoders.getCountsRight()-startRight);
                line((float)diff, 200);
                delay(1);
            }
            delay(500);
           
            rotate(90);
            delay(10);
            rotate(90);
            
            startLeft = encoders.getCountsLeft();
            startRight = encoders.getCountsRight();

            timer = millis();
            while (millis() - timer < 2000)
            {
                int diff = (encoders.getCountsLeft()-startLeft) - (encoders.getCountsRight()-startRight);
                line((float)diff, 200);
                delay(1);
            }
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
    public:

        byte command()
        {
            //Vil prøve å få på rolling på LCD, slik at me kan skrive lengre instruksar...

            static byte config = 0;

            String selection[] = {
                "Calibrate linesensors",
                "Line follower",
                "Object follower",
                "Square",
                "Sircle",
                "Back and forth",
                "Slalom"    
            };

            if (buttonB.getSingleDebouncedRelease()) {
                while (true) {
                    lcd.clear();
                    lcd.print("B to continue");
                    lcd.gotoXY(0, 1);
                    lcd.print("A or C to configure");

                    if (buttonB.getSingleDebouncedRelease()) return config;
                    if (buttonA.isPressed() && buttonC.isPressed()) break;
                } 
            }
            
            if (buttonA.isPressed() || buttonC.isPressed()) {
                buttonA.waitForRelease();
                buttonC.waitForRelease();

                while (true) {
                    lcd.clear();
                    lcd.print(selection[config]);
                    lcd.gotoXY(0, 1);
                    lcd.print("<A B^ C>");

                    if (buttonA.getSingleDebouncedRelease()) config--;
                    if (buttonB.getSingleDebouncedRelease()) break;
                    if (buttonC.getSingleDebouncedRelease()) config++;
                    if (config < 0) config = sizeof(selection) - 2; //Må hardkode sidan kvart element i selection har ulik mengde bytes
                    if (config > sizeof(selection) - 2) config = 0;
                }
            }
            
            if (usbPowerPresent()) {
                if (!Serial) Serial.begin(9600);
                while (!Serial);
                //Vis instruksjonar

                if (Serial.available()) {
                    switch (Serial.read()) {
                        case 'q':
                            //Konfigurer
                            break;
                        case 'B':
                            //Konfigurer
                            break;
                        default:
                            break;
                    }
                }
            }

            return config;
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
};

SelfDriving drive;                  //Instans for sjølvkjøring
Interface intf;


void setup() 
{
    /*buttonA.waitForPress();
    delay(1000);
    drive.calibrateSensors();*/
}



void loop()
{
    buttonA.waitForPress();
    delay(1000);

    drive.rot(90);
    delay(10);
    drive.rot(90);

    buttonA.waitForPress();
    delay(1000);
    
    drive.rot(180);

    buttonA.waitForPress();
    delay(1000);

    drive.backAndForth();

    buttonA.waitForPress();
    delay(1000);

    drive.square();
}