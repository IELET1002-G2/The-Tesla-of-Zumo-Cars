#include <Zumo32U4.h>               //Importerer Zumo-biblioteket
#include <Wire.h>
#include <LSM303.h>

Zumo32U4Motors motors;              //Oppretter instans av motorane
Zumo32U4Encoders encoders;          //Oppretter instans av kodarane
Zumo32U4LineSensors lineSensors;    //Oppretter instans av linjesensorane
Zumo32U4ButtonA buttonA;            //Oppretter instans av knapp A




class SelfDriving
{
    private:

        unsigned int lineSensorValues[5];   //Verdien til kvar linjesensor

        int leftSpeed;
        int rightSpeed;

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

            motors.setSpeeds(0, 0);
            delay(50);
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

        int PD(int input, int last, int speed, int batteryLevel)
        {
            int error = 2000 - input;                                   //Converts position to error based on desired position
            float batteryCorr = 1.00E+00 - exp(-1.00E-01*batteryLevel); //Correction for battey level

            int adjust = 0.4*error + 2.0*(error-last);                  //Adjustment based on error and deriavative

            int left = constrain(speed - adjust, -400, 400);            //Left motor speed based on adjustment
            int right = constrain(speed + adjust, -400, 400);           //Right motor speed based on adjustment

            motors.setSpeeds(left*batteryCorr, right*batteryCorr);      //Set speeds adjusted for battery level

            return error;                                               //Return error for next deriavative
        }

    public:

        void rot(int d) {rotate(d);}

        void square()
        {
            for (byte n = 0; n < 4; n++) {
                motors.setSpeeds(200, 200);
                delay(2000);
                rotate(90);
            }
        }

        void line(unsigned long time) 
        {
            unsigned long start = millis();                             //Start time

            int leftCounter = encoders.getCountsLeft();                 //Start counts left
            int rightCounter = encoders.getCountsRight();               //Start counts right

            while (millis() - start < time)                             //Drives forward for set amount of time
            {
                static int last = 0;
                last = encoderPD(leftCounter, rightCounter, last);      //Adjusts motor speeds and stores return value
            }
            motors.setSpeeds(0,0);                                      //Stops motors at end of time period
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

        void followLinePD(int speed, int batteryLevel)
        {
            static int last = 0;
            int position = lineSensors.readLine(lineSensorValues);  //Reads position from lineSensors
            last = PD(position, last, speed, batteryLevel);         //Adjusts motors based on position and stores return value
        }

};

SelfDriving drive;                  //Instans for sjølvkjøring


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
    drive.square();
    
}