#include <Zumo32U4.h>                   //Importerer Zumo-biblioteket

Zumo32U4Motors motors;                  //Oppretter instans av motorane
Zumo32U4Encoders encoders;              //Oppretter instans av kodarane
Zumo32U4LineSensors lineSensors;        //Oppretter instans av linjesensorane
Zumo32U4ButtonA buttonA;                //Oppretter instans av knapp A
Zumo32U4LCD lcd;                        //Oppretter instans av LCD-display
Zumo32U4Buzzer buzzer;                  //Oppretter instans av buzzeren
Zumo32U4ProximitySensors proxSensors;   //Create instance of the proximity sensors

unsigned int lineSensorValues[5];       //Verdien til kvar linjesensor



class SelfDriving
{
    private:

        bool sensorInitInterlock;                          //Bool to save interlock value through program                    
 
 
    public:

        void calibrateSensors() 
        {
            sensorInitInterlock = false;                    //Linesensors is now configured, no need to do it again 
                                                            //unless prox. sensors config has been run 
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
            if (sensorInitInterlock) { 
                lineSensors.initFiveSensors(); 
 
                sensorInitInterlock = false;                            //If followObject() is called after followLine() has been called, initFrontSensor  
            } 
 
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
            
            motors.setSpeeds(leftSpeed, rightSpeed);    //Setter fart til utrekna, korrigerte verdiar
        }


        void followObject()                             //Follow an object within prox. sensor range (~30-40 cm) as it would follow lines on ground
        {
            if (!sensorInitInterlock) { 
                proxSensors.initFrontSensor();          //Need to call this function in order to use the front prox. sensor
                                                        //Can only use front when full line sensor array in use, ref. p. 20 in Pololu User Guide
                sensorInitInterlock = true;             //Init interlock, if followLine() is called after followObject() has been called, initFiveSensors
            }                       

            int driveSpeed = 150;
            int turnSpeed = 300;

            proxSensors.read();                                          //Function that reads reflected IR from nearby object, takes ~ 3 ms to run

            int leftReading = proxSensors.countsFrontWithLeftLeds();     //Returns an integer based on reflected IR light
            int rightReading = proxSensors.countsFrontWithRightLeds();

            while (((leftReading + rightReading)) / 2 < 4) {             //As long as no object is detected (object is ~ 140 cm from Zumo), turn until detected
                proxSensors.read();                                      //Read sensor values for every iteration of the while loop

                motors.setSpeeds(turnSpeed, 0);

                leftReading = proxSensors.countsFrontWithLeftLeds();     //Get value from front left prox. sensor
                rightReading = proxSensors.countsFrontWithRightLeds();   //Get value from front right prox. sensor

                if(leftReading >= 4 || rightReading >= 4) {              //If the sensor detected something (within ~ 140 cm range), stop turning and exit while loop
                    motors.setSpeeds(0,0);
                    delay(30);                                           //Motor safety delay time
                }
            }

            if ((leftReading > 5) || (rightReading > 5)) motors.setSpeeds(0, 0);                     //Object must be very close (less than 30 cm) to an object, stop motors as a preventive measure 

            else if (leftReading > rightReading) motors.setSpeeds(driveSpeed, turnSpeed);            //Object is to the left, turn left 

            else if (leftReading < rightReading) motors.setSpeeds(turnSpeed, driveSpeed);            //Objct is to the right, turn right

            else motors.setSpeeds(driveSpeed, driveSpeed);                                           //Object is directly in front of Zumo, drive towards object 
        }
};




SelfDriving drive;                  //Instans for sjølvkjøring


void setup()
{  
    delay(1000);
    drive.calibrateSensors();                                   //Kalibrerer sensorane på kommando
}



void loop()
{
    unsigned long time = millis();

    int position = lineSensors.readLine(lineSensorValues);      //Leser av posisjonen til zumoen

    if (time < 10000) drive.followLine(position, false, 100);             //Korrigerer retning basert på posisjon

    else drive.followObject();
}


