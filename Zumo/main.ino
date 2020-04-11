#include <Zumo32U4.h>                                                   //Include different libraries for use with Zumo
#include <Wire.h>
#include <LSM303.h>

Zumo32U4Motors motors;                                                  //Create instances of the different objects of the Zumo
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4LCD lcd;
Zumo32U4Buzzer buzzer;
L3G gyro;
Zumo32U4ProximitySensors proxSensors;




/**
 * Class containing methods for different driving configurations of the Zumo32U4.
 * Some methods include PD-regulated driving configurations, while others do not.
 * The methods are all coded to serve their own purpose, so depending on its purpose,  
 * line sensors, motor encoders, the gyro and the front proximity sensors are
 * applied where applicable. Should only declare object once.
*/
class SelfDriving
{
    private:

        unsigned int lineSensorValues[5];                               //Declare array to store the five readings from line sensors

        bool sensorInitInterlock;                                       //Bool as interlock when switching between followLine and followObject
        int leftSpeed;
        int rightSpeed;
        long gyroNoise = 0;
        const int threshold = 300;                                      //Threshold for line sensors


        /**
         * Method that uses the motor encoders to rotate the Zumo to
         * the set number of degrees given in method argument.
        */
        void rotate(int degrees)
        {
            float arcCounts = 7.63125*degrees;                          //Calibrated number of counts for given angle

            int leftStart = encoders.getCountsLeft();                   //Start counts left
            int rightStart = encoders.getCountsRight();                 //Start counts right

            int sum = 0;

            motors.setSpeeds(0, 0);                                     //Stop motor before rotation
            delay(50);                                                  //Delay to get rid of momentum

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


        /**
         * Method that uses the Zumo's gyro to PLEASE HELP HERE.
         * STRUGGLING TO PROVIDE ACCURATE DESCRIPTION OF WHAT THIS
         * METHOD DOES.
        */
        long line(long integral, int speed)
        {
            while(!gyro.readReg(gyro.STATUS_REG));                      //Wait for new available
            gyro.read();                                                //Read latest gyro data

            long g = gyro.g.z - gyroNoise;

            if (g > 80 || g < -80)
            {
                integral += g;
                int adjust = 0.015 * g + 0.002 * integral;
                motors.setSpeeds(speed + adjust, speed - adjust);
            }
            return integral;
        }


        /**
         * A PD-regulated driving configuration method that uses the 
         * line sensnor values as input for the regulator.
        */
        int PD(int input, int last, int speed, int batteryLevel = 100, bool emergencyPower = false)
        {
            int error = 2000 - input;                                   //Converts position to error based on desired position
            float batteryCorr = 1.00E+00 - exp(-1.00E-01*batteryLevel); //Correction for battery level

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


        /**
         * Method that uses the line sensor values to return a bool
         * if a right turn or a left turn is detected.
        */
        bool turnDetect()
        {
            static unsigned long lastTurn = 0;
            lineSensors.readLine(lineSensorValues);
            if (lineSensorValues[0] > threshold || lineSensorValues[4] > threshold)
            {
                lastTurn = millis();
                return true;
            }
            if (millis() - lastTurn < 300) return true;
            return false;
        }


        /**
         * A method that uses the line sensor values and a threshold to
         * return true/false whether the Zumo is above a black line on
         * the floor or not.
        */
        bool noLine()
        {
            if (
                lineSensorValues[0] < threshold &&                      //While each value is under threshold, return true
                lineSensorValues[1] < threshold &&
                lineSensorValues[2] < threshold &&                      //If line is found, return false
                lineSensorValues[3] < threshold &&
                lineSensorValues[4] < threshold
            ) {
                return true;
            }
            return false;

        }
        
        
        /**
         * Method that returns true or false based on whether a black 
         * line is found or not. It uses other methods such as turnDetect()
         * and noLine() to drive straight forward, rotate 180 degrees and
         * drive back.
        */
        bool noLineFound()                                              //Car has 2 seconds to find, and find back to, line if no line is detected
        {
            const unsigned long timeThreshold = 2000;                   //Time interval before car should rotate 180 deg and go back
            static unsigned long timeStart = millis();                  //Time since no line was found
            static long integral = 0;

            
            if (turnDetect() || !noLine()) {
                timeStart = millis();                                   //If line is found, update timeStart so car does not turn around
                integral = 0;
                return false;
            }

            if (millis() - timeStart > timeThreshold)                   //If car has not found line in 2 seconds, rotate 180 deg and drive straight back
            {
                rotate(180);
                timeStart = millis();                                   //Update timeStart so car does not turn around again while still no line is found
                integral = 0;
                motors.setSpeeds(200, 200);
            }
            integral = line(integral, 250);
            return true;
        }


        /**
         * Method that calibrates the line sensors and the gyro.
         * This method should be called before any other method
         * that depends on the line sensor/gyro is called. Examples
         * are followLine() and line().
        */
        void calibrateSensors() 
        {
            sensorInitInterlock = false;                                //Interlock bool to keep Zumo from initiating line sensors 
                                                                        //every time followLine method is called
            lineSensors.initFiveSensors();                              //Initiate the five line sensors on front array of Zumo

            Wire.begin();
            gyro.init();
            gyro.enableDefault();

            delay(500);

            for (int i = 0; i < 100; i++)
            {
                while(!gyro.readReg(gyro.STATUS_REG));
                gyro.read();
                gyroNoise += gyro.g.z; 
            }
            gyroNoise /= 100;

            for (int t = 0; t <= 200; t++) {                            //For loop that lasts for 4000 ms where t is time in ms

                lineSensors.calibrate();                                //Calibrates line sensors for every iteration of loop

                int speed = 400*sin(PI/100*t);                          //Rotation of car is described by a sine function where T = 4000 ms
                
                motors.setSpeeds(speed, -speed);                        //Zumo rotates with speed depending on sine function
                delay(1);                                               //Delay to manage period time
            }
        }


        /**
         * Method that uses the line sensor values and a predefined 
         * polynomial to regulate motor speeds. Is used to follow 
         * black line on ground.
        */
        void followLine(int batteryLevel, bool emergencyPower = false, bool fastMode = false)
        {
            if (sensorInitInterlock) { 
                lineSensors.initFiveSensors(); 

                sensorInitInterlock = false;                            //If followObject() is called after followLine() has been called, initiate front prox. sensor  
            } 

            int value = lineSensors.readLine(lineSensorValues);         //Gets line sensor readings from array and returns an int from 0-4000 
            float batteryCorr = 1.00E+00 - exp(-1.00E-01*batteryLevel); //Correction factor for battery level

            if (fastMode) {                                             //Fast mode
                leftSpeed = 4.00E+02 - 8.00E+02*exp(-2.50E-03*value);   //Speed is set by exponential functions
                rightSpeed = 4.00E+02 - 3.63E-02*exp(+2.50E-03*value);
            }
            else {
                leftSpeed =                                             //Left motor speed is equal to a polynomial to the 4th degree where the variable 
                    -4.00E+02*pow(value, 0)                             //is the sensor value with Df = [0, 4000].
                    +1.04E+00*pow(value, 1)                             //The polynomial is icreasing in Df with values 
                    -6.63E-04*pow(value, 2)                             //Vf = [-400, 400].
                    +1.80E-07*pow(value, 3)                             //There is a saddle point at (2000, 200).
                    -1.67E-11*pow(value, 4);

                rightSpeed =                                            //Right motor speed is equal to a polynomial to the 4th degree where the variable
                    +4.00E+02*pow(value, 0)                             //is the sensor value with Df = [0, 4000].
                    -1.07E-01*pow(value, 1)                             //The polynomial is decreasing in Df with values 
                    -1.03E-04*pow(value, 2)                             //Vf = [-400, 400].
                    +8.67E-08*pow(value, 3)                             //There is a saddle point at (2000, 200).
                    -1.67E-11*pow(value, 4);
            }

            leftSpeed *= batteryCorr;                                   //Correct the speed according to the battery level
            rightSpeed *= batteryCorr;

            if (batteryLevel <= 10 && !emergencyPower) {                //If battery level is 10 % and emergency power is not activated
                leftSpeed = 0;                                          //Set the two motor speeds to 0
                rightSpeed = 0;
            }
            
            motors.setSpeeds(leftSpeed, rightSpeed);                    //Set corrected motor speeds to motors
        }


        /**
         * Method that makes the Zumo follow black line on ground
         * using line sensor values and PD-regulation.
        */
        void followLinePD(int s, int batteryLevel)
        {
            static int last = 0;
            int p = lineSensors.readLine(lineSensorValues);             //Reads position from lineSensors
            last = PD(p, last, s, batteryLevel);                        //Adjusts motors based on position and stores return value
        }


        /**
         * This method makes the Zumo follow an object that reflects
         * IR rays well. It uses the two front IR diodes on the car to
         * transmit IR rays on a specified pulse to the environment, and 
         * uses the front proximity sensor to interpret its surroundings.
         * When an object is within range (~ 30-40 cm), it follows that
         * object. Otherwise scans if no object within range.
        */
        void followObject()
        {
            if (!sensorInitInterlock) { 
                proxSensors.initFrontSensor();                          //Need to call this function in order to use the front prox. sensor
                                                                        //Can only use front prox. sensor when full line sensor array in use
                sensorInitInterlock = true;                             //Init interlock, if followLine() is called after followObject() has been called, initFiveSensors
            }                       

            int driveSpeed = 150;
            int turnSpeed = 300;

            proxSensors.read();                                         //Function that reads reflected IR from nearby object, takes ~ 3 ms to run

            int leftReading = proxSensors.countsFrontWithLeftLeds();    //Returns an integer based on reflected IR light
            int rightReading = proxSensors.countsFrontWithRightLeds();

            while (((leftReading + rightReading)) / 2 < 4) {            //As long as no object is detected (object is ~ 140 cm from Zumo), turn until detected
                proxSensors.read();                                     //Read sensor values for every iteration of the while loop

                motors.setSpeeds(turnSpeed, 0);

                leftReading = proxSensors.countsFrontWithLeftLeds();    //Get value from front left prox. sensor
                rightReading = proxSensors.countsFrontWithRightLeds();  //Get value from front right prox. sensor

                if(leftReading >= 4 || rightReading >= 4) {             //If the sensor detected something (within ~ 140 cm range), stop turning and exit while loop
                    motors.setSpeeds(0,0);
                    delay(30);                                          //Motor safety delay time
                }
            }

            if ((leftReading > 5) || (rightReading > 5)) motors.setSpeeds(0, 0);                     //Object must be very close (less than 30 cm) to an object, stop motors as a preventive measure 

            else if (leftReading > rightReading) motors.setSpeeds(driveSpeed, turnSpeed);            //Object is to the left, turn left 

            else if (leftReading < rightReading) motors.setSpeeds(turnSpeed, driveSpeed);            //Objct is to the right, turn right

            else motors.setSpeeds(driveSpeed, driveSpeed);                                           //Object is directly in front of Zumo, drive towards object 
        }


        /**
         * Method that makes the Zumo drive in a square pattern.
        */
        void square()
        {
            for (byte n = 0; n < 4; n++) {
                long integral = 0;
                unsigned long timer = millis();

                motors.setSpeeds(200, 200);
                while (millis() - timer < 1000)  integral = line(integral, 200);

                motors.setSpeeds(0,0);
                delay(500);
                rotate(90);
            }
        }


        /**
         * Method that makes the Zumo drive in a circle pattern.
        */
        void circle()
        {
            motors.setSpeeds(300, 150);
            delay(3000);
            motors.setSpeeds(0, 0);
        }


        /**
         * Method that makes the Zumo drive in back and forth in 
         * a straight line Drives forwards for 2000 ms, rotates
         * 180 degrees and drives back.
        */
        void backAndForth()
        {
            for (byte i = 0; i < 2; i++)
            {
                long integral = 0;
                unsigned long timer = millis();

                motors.setSpeeds(200, 200);
                while (millis() - timer < 2000) integral = line(integral, 200);
                motors.setSpeeds(0, 0);
            
                rotate(180);
            }
        }


        /**
         * Method that makes the Zumo drive slalom between 10 cones
         * with a predefined spacing between each cone.
        */
        void slalom()
        {
            motors.setSpeeds(0, 0);
            delay(50);
            
            int counts = encoders.getCountsLeft();
            float coneCounts = 0.5*7425;                                //Cone distance of 0.5m
            unsigned long time = millis();

            while (encoders.getCountsLeft() - counts < coneCounts) {
                motors.setSpeeds(200, 200);
            }

            time = millis() - time;
            
            for (byte i = 0; i < 10; i++){                              //10 cones
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


/**
 * Containing methods for interfacing with Zumo32U4 car including graphical user 
 * inteface with buttons, LCD-display and Serial monitor. Also includes more advanced
 * printing functions than native to the Zumo32U4LCD class. Should only have one 
 * object and should not be inherited. Class only sorts similar and related functions
*/
class Interface
{
    private:

        bool forceConfig = true;                                        //Variable that controls whether or not buttons are required to prompt configuration.
        int config[2] = {0, 0};                                         //Array stores mode and configuration.

        /**
         * Pauses program if button B is pressed and gives option of going straight 
         * to configuration if button A or C is pressed, or continuing current mode 
         * and configuration if button B is pressed. Intended for command()
        */
        void pause() {
            if (buttonB.getSingleDebouncedRelease()) {                  //Pauses if button B is pressed.
                motors.setSpeeds(0, 0);
                print("B:cont", "A/C:conf");
                
                while (true) {                                          //Continuously checks if somthing happens.
                    if (buttonB.getSingleDebouncedRelease()) break;     //Continues if button B is pressed again.
                    if (releasedAorC()) {                               //Any other button prompts configuration.
                        enableForceConfig();
                        break;                                                  
                    }
                } 
            }
        }

        /**
         * Statemachine that that reads whether button A or button C have gone from
         * pressed state to released state. Intended for command() and pause()
         * 
         * @returns true if in right state, else false
        */
        bool releasedAorC() {
            return                                                      //or not A or C have been pressed.
                buttonA.getSingleDebouncedRelease() ||
                buttonC.getSingleDebouncedRelease();
        }

        /**
         * Toggles between modes and configurations in config array. Only intended
         * for command()
         * 
         * @param menu the index of config where value is toggled
         * @param inc the increment (or decrement) of which the value is toggled
         * @param max the smallest value in config before it toggles back to start
        */
        void toggleConfig(byte menu, byte inc, byte max) {
            if (buttonA.getSingleDebouncedRelease()) config[menu] -= inc;   //Toggles back to previous mode.
            if (buttonC.getSingleDebouncedRelease()) config[menu] += inc;   //Toggles forward to following mode.
            if (config[menu] < 0) config[menu] = max;                       //Toggles to rightmost configuration.
            if (config[menu] > max) config[menu] = 0;                       //Toggles to leftmost configuration.
        }

        /**
         * Reads incoming serial buffer if serial receive buffer is not empty, and
         * waits for buffer being emptied. Only intended for command()
         * 
         * @param menu the index of config array where the data is stored
         * @returns true if buffer was emptied, false if buffer already was empty
        */
        bool getSerial(byte menu) {
            if (Serial.available()) {
                Serial.flush();                                         //Waits until incoming buffer can be read.
                config[menu] = Serial.parseInt();                       //Gets mode or configuration from monitor.
                return true;                                            //Controls flow if configuration received.
            }
            else return false;
        };


    public:

        /**
         * User interface for choosing mode and configure mode through either 
         * buttons and LCD-display, or serial monitor
         * 
         * Button A toggle back
         * Button B confirm
         * Button C toggle forward
         * 
         * @returns config a pointer (array of integers) storing mode (index 0)
         *      with values from 0 to 6, and the configuration of the mode (index 1)
        */
        int* command() {
            const char* MODES[] = {                                     //Array with names of modes.
                "Calib",    //0: Calibrate line sensors
                "Line",     //1: Follow line
                "Object",   //2: Follow object
                "Square",   //3: Drive in square pattern
                "Circle",   //4: Drive in cirular pattern
                "B and F",  //5: Drive forwards, turn 180D, go back
                "Slalom"    //6: Slalom between cones
                };
            const char* LINE_CONFIG[] = {                               //Array with line follower configurations.
                "Normal",   //0: Polynomial regulated
                "PD"        //1: PD regulated
                };
            
            pause();                                                    //Pauses if button B is pressed.

            if (releasedAorC() || forceConfig) {                        //Prompts selection of modes.
                config[1] = 0;                                          //Resets configuration.
                
                if (usbPowerPresent()) {
                    Serial.begin(9600);                                 //Initiates serial monitor.
                    while(!Serial);                                     //Waits for serial monitor to open.
                }

                if (Serial) {
                    Serial.print(                                       //Prints selection of modes on monitor.
                        "Select mode:\n"
                        "   0: Calibrate linesensors\n"
                        "   1: Line follower\n"
                        "   2: Object follower\n"
                        "   3: Square\n"
                        "   4: Circle\n"
                        "   5: Back and forth\n"
                        "   6: Slalom\n\n"
                        );
                }

                while (Serial.available()) {                            //Flush incoming buffer.
                    Serial.flush();
                    Serial.parseInt();
                }

                while (true) {                                                  //Continuously checks if somthing happens.                                                     
                    print(MODES[config[0]], "<A B^ C>");                        //Prints current mode.
                    toggleConfig(0, 1, 6);                                      //Toggles mode.                                                    
                    if (buttonB.getSingleDebouncedRelease() || getSerial(0)) break;  //Chooses current mode.  
                }

                if (Serial) {
                    switch (config[0]) {
                        case 1:
                            Serial.print(                                       //Prints selection of configurations.
                                "Configure mode:\n"
                                "   0: Normal\n"
                                "   1: PD-regulated\n\n"
                                "Selected configuration: "
                                );
                            break;
                        case 6:
                            Serial.print("Enter centimeters between cones: ");  //Asks for distance.
                            break;
                        default:
                            Serial.print("Selected mode: ");
                            Serial.print(config[0]);
                    }
                }

                while (config[0] == 1 || config[0] == 6) {
                    if (config[0] == 1) {                                       //Prompts line follower configuration.
                        print(LINE_CONFIG[config[1]], "<A B^ C>");              //Prints current configuration.
                        toggleConfig(1, 1, 1);                                  //Toggles configuration.
                    }   
                    if (config[0] == 6) {                                       //Prompts slalom configuration.
                        print("cm: ", "<A B^ C>");  
                        print(config[1], 4, 0);                                 //Prints current distance between cones.
                        toggleConfig(1, 10, 100);                               //Toggles configuration.
                    }
                    if (buttonB.getSingleDebouncedRelease() || getSerial(1)) {
                        if (Serial) Serial.print(config[1]);                    //Prints confirmation to monitor.
                        break;                                                  //Chooses current configuration.
                    }
                }

                if (Serial) {                                           //Prints confirmation to monitor.
                    Serial.print(
                        "\n\nConfiguration done.\n"
                        "Press button B to confirm.\n\n"
                        );
                }

                print("Ready.", "Press B");
                buttonB.waitForRelease();                               //Wait for button B to be pushed.
                lcd.clear();
            }
            
            forceConfig = false;                                        //Turns of forcing.
            return config;                                              //Returns pointer to array that stores configuration.
        }
        
        /**
         * Prompts command() to run without any button being pressed
        */
        void enableForceConfig() {
            forceConfig = true;                                         //Forces to configure a single time after call.
        }

        /**
         * Timed print function to reduce interlace
         * 
         * @param value an integer to be printed to LCD-display
         * @param X, Y the coordinates where the integer is printed
        */
        void print(int value, int X, int Y) {
            static unsigned long timer = millis();                      //Variable that stores last time the function printed.

            if (millis() - timer > 100) {                               //Prints less frequently than every 100ms.
                lcd.gotoXY(X, Y);                                       //Prints to chosen position.
                lcd.print(value);                                       //Prints number without decimals.
                
                timer = millis();                                       //Saves time.
            }
        }

        /**
         * Timed print function to reduce interlace
         * 
         * @param value a floating-point number to be printed to LCD-display
         * @param X, Y the coordinates where the floating-point number is printed
        */
        void print(float value, int X, int Y) {
            static unsigned long timer = millis();                      //Variable that stores last time the function printed.

            if (millis() - timer > 100) {                               //Prints less frequently than every 100ms.
                lcd.clear();
                lcd.gotoXY(X, Y);                                       //Prints to chosen position.
                lcd.print(value);                                       //Prints number with decimals.
                
                timer = millis();                                       //Saves time.
            }
        }

        /**
         * Timed print function to reduce interlace
         * 
         * @param string1 a string to be printed to first row on LCD-display
         * @param string2 a string to be printed to second row on LCD-display
        */
        void print(char* string1, char* string2) {
            static unsigned long timer = millis();                      //Variable that stores last time the function printed.

            if (millis() - timer > 100) {                               //Prints less frequently than every 100ms.
                lcd.clear();
                lcd.print(string1);                                     //Prints text to first row.
                lcd.gotoXY(0, 1);
                lcd.print(string2);                                     //Prints text to second row.
                
                timer = millis();                                       //Saves time.
            }
        }

        /**
         * Prints string to LCD-display where the first 8 characters are displayed 
         * on the first row and additional characters are displayed on the second row
         * 
         * @param message a string to be printed on LCD-display
        */
        void printMessage(char message[]) {
            lcd.clear();

            for (byte x = 0; x < constrain(sizeof(message)-1, 0, 16); x++)  //Iterates through string.
            {
                if (x >= 8) {                                           //If string is longer than first row.
                    lcd.gotoXY(x-8, 1);                                 //Goes to second row and resets coulumn.
                    lcd.print(message[x]);                              //Prints character.
                }
                else {
                    lcd.gotoXY(x, 0);                                   //Goes to new coulumn.
                    lcd.print(message[x]);                              //Prints character.
                }
            }
        }     
};



/**
 * Class containing methods for calculation of speed and displacement. These methods
 * rely on the motor encoders for calculations. Should only declare object once.
*/
class Motion
{
    private:

        float momSpeed;
        float avgSpeed;
        float trip = 0.0;
        float distance = 0.0;
        float displacement = 0.0;


        /**
         * Method that calculates trip, distance and displacement.
        */
        void calculateMotion()
        {
            static unsigned long timer;                                 //Timer to save time interval

            int leftCount = encoders.getCountsAndResetLeft();           //Reads the left encoder's value
            int rightCount = encoders.getCountsAndResetRight();         //Reads the right encoder's value

                                                                        //Theoretical count per meter is 909.7(2*pi*r)=7425
            float avgDisp = (leftCount + rightCount)/(2.0*7765.0);      //Average displacement of the Zumo
            float avgDist = abs(avgDisp);                               //Average distance is the absolute value of the displacement

            trip += avgDist;                                            //Accumulate trip-counter
            distance += avgDist;                                        //Accumulate distance
            displacement += avgDisp;                                    //Accumulate displacement

            momSpeed = avgDisp/(millis() - timer)*1000;                 //Momentary speed
            timer = millis(); 
        }


        /**
         * Method that calculates the Zumo's average speed.
        */
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

        /**
         * Method that calculates trip, distance and displacement,
         * and returns momentary speed.
        */
        float getSpeed()
        {
            calculateMotion();                                          //Calculates the motion
            return momSpeed;                                            //Returns the momentary speed
        }


        /**
         * Method that calculates trip, distance and displacement,
         * and returns distance.
        */
        float getDistance()
        {
            calculateMotion();                                          //Calculates the motion
            return distance;                                            //Returns the distance
        }


        /**
         * Method that calculates trip, distance and displacement,
         * and returns displacement.
        */
        float getDisplacement()
        {
            calculateMotion();                                          //Calculates the motion
            return displacement;                                        //Returns the displacement
        } 
        

        /**
         * Method that calculates trip, distance and displacement,
         * and returns trip.
        */
        float getTrip()
        {
            calculateMotion();                                          //Calculates the motion
            return trip;                                                //Returns the trip counter
        }

        /**
         * Method that sets trip value.
        */
        void setTrip(int value)
        {
            trip = value;                                               //Sets trip value
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
            static bool ledState = HIGH;                                //Variable that saves LED state
            
            for (byte i = 0; i <= 100; i++) {
                level = i;

                ledRed(ledState);
                ledState = !ledState;                                   //Toggles the state
                delay(400);
            }
        }


        int getBatteryLevel(float trip, float weight=0)
        {
            static float lastTrip = 0.0;

            level -= (trip-lastTrip)*(275+weight)/275;                  //Calculates the battery level
            lastTrip = trip;                                            //Saves the last trip

            if (level <= 10) {                                          //If the battery level is below 10 %
                empty = true;                                           //Incremens the trip counter
                ledRed(HIGH);
            }

            return constrain(level, 0, 100);                            //Returns battery level
        }
    

        bool getEmergencyPower()
        {
            return empty;                                               //Returns remaining battery level first time under 10 %
        }


        void resetEmpty()
        {
            empty = false;
        }
};



SelfDriving drive;                                                      //Create object of the four classes
Interface intf;
Motion motion;
Battery battery;



void setup() {}



void loop()
{
    float distance;
    int batteryLevel;

    int* config = intf.command();

    switch (config[0]) {
        case 0:
            drive.calibrateSensors();                                   //Calibrates the sensors on command
            intf.enableForceConfig();
            break;

        case 1:
            distance = motion.getTrip();                                //Gets distance driven
            batteryLevel = battery.getBatteryLevel(distance);           //Gets battery level based on distance driven

            if (drive.noLineFound());
            else if (config[1] == 0) drive.followLine(batteryLevel);    //Corrects direction based on position
            else drive.followLinePD(250, batteryLevel);

            intf.print(distance, 0, 0);                                 //Prints position to first line on LCD
            intf.print(batteryLevel, 0, 1);                             //Prints battery level to second line on LCD
            break;

        case 2:
            drive.followObject();                                       //Calls follow object mode
            break;
        
        case 3:
            drive.square();                                             //Calls square method
            intf.enableForceConfig();                                   //Triggers interface menu
            break;

        case 4:
            drive.circle();                                             //Calls circle method
            intf.enableForceConfig();                                   //Triggers interface menu
            break;

        case 5:
            drive.backAndForth();                                       //Calls backAndForth method
            intf.enableForceConfig();                                   //Triggers interface menu
            break;

        case 6:
            drive.slalom();                                             //Calls slalom method
            intf.enableForceConfig();                                   //Triggers interface menu
            break;

        default:
            break;
    }
}