package com.qualcomm.ftcrobotcontroller.opmodes.BombSquadOpModes;

//import OpModes

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

/* Made by the programmers of FTC Team 9804 Bomb Squad
 *v1 3-2-16 at 7:58 pm Steve & Etienne -- created new Oakland tournament code with additional servos
 *v1 3-5-16 at 5:06 pm Etienne L. -added all clear servo
 *v2 3-11-16 at 9:00 pm Etienne L. --added comments from teleop comments
 *v2 3-15-16 at 12:00 pm Etienne L. --added comments from teleop comments
 *v2 3-15-16 @ 5:27 pm Etienne L. --added gamepad comments
 *v2 3-17-16 @ 5:59 pm Etienne L. --changed hopper variable names & beacon dump
 *      -> edited up to lines 123-126 On OpMode Teleop v1
 *v2 3-18-16 @ 4:54 pm Etienne L --addressing comments
 *v3 3-19-16 @ 10:50 am Etienne L. --starting version 3 with continuous gain
 *v5 3/20 @1:40 --
 */

//////////////~~~ALL GAMEPAD CONTROLS~~~//////////////
//true as of March 15th 2016 @ 4:44 PM
 /*
*************DRIVER************* #1
-Left Trigger             ->    left grabber down (YES)
-Right Trigger            ->    right grabber down (YES)
-Left Bumper              ->    left grabber up (YES)
-Right Bumper             ->    right grabber up (YES)
-Dpad UP                  ->    arms extend (time phased power) (YES)
-Dpad DOWN                ->    arms retract (time phased power) (YES)
-Dpad RIGHT               ->    hook poles forward (NO)
-Dpad LEFT                ->    hook poles back (NO)
-|Y|ellow Button          ->    all clear function (pending) (NO)
-|X|enon (blue) Button    ->    hopper run right (LOOKS LIKE LEFT)
-|B|rick (red) Button     ->    hopper run left (LOOKS LIKE RIGHT)
-|A|pple (green) Button   ->    window wiper (YES)
-Right Joystick           ->    right tread (forward [+ after reversal]) (backward [-after reversal]) (YES, y axis)
-Left Joystick            ->    left tread (forward [+] backward [-]) (Yes, y axis)
*************GUNNER************* #2
-Left Trigger             ->    left winch in (set to hang arms to in)&
-Right Trigger            ->    right winch in (set hang arms to in)&
-Left Bumper              ->    left winch out (set hang arms to out)&
-Right Bumper             ->    left winch out (set hang arms to out)&
-Dpad UP                  ->    shelter drop forward (arms)
-Dpad DOWN                ->    shelter drop back (arms)
-Dpad RIGHT               ->    move zipline bar right (all clear up)
-Dpad LEFT                ->    move zipline bar left (all clear down) zipline bar is s(8) >> 2 states, hold 0 and release 1... right is release left is hold
-|Y|ellow Button          ->    eject debris (YES)
-|X|enon (blue) Button    ->    select blue alliance (YES)
-|B|rick (red) Button     ->    select red alliance (YES)
-|A|pple (green) Button   ->    collect debris (YES)
-Right Joystick           ->    arms extend (up[-]) and retract (down[+]) (right winch)
-Left Joystick            ->    NOT USED (left winch)
 */

public class Oak_9804_TeleOp_v5 extends OpMode {

    //defining the motors, servos, and variables in this program

    //magnetic sensor
    DigitalChannel sensorExtend;        // detects magnet, arms fully extended
    DigitalChannel sensorRetract;       // detects magnet, arms fully retracted
    DigitalChannel extendLED;           // these are indicator LEDs
    DigitalChannel retractLED;          // that will signal the drivers when arm limits are reached

    //drive motors
    DcMotor driveLeftBack;               // there are 2 motors functioning in each tread and the tread
    DcMotor driveLeftFront;              //requires that the leading or front motor is powered at 95% of what
    DcMotor driveRightBack;              // the Back or trailing motor is powered. Further on the left motors
    DcMotor driveRightFront;             // are reversed because the right side goes forward with positive values.
    // back or front DOES depend on which direction you are driving
    //this is addressed later on in one statement while avoiding the joystick
    //deadzones.

    //winch motors
    DcMotor leftWinch;                   //in this version of TeleOp code there are no functions to control the
    DcMotor rightWinch;                  //speed of the winch because the hooks will no longer deploy early if the
    //speed of the arms exceeds the speed of the winch

    //motors for extending arms and spinner (debris collector)
    DcMotor arms;
    DcMotor spin;

    //servos to lock in place on ramp
    Servo grabLeft;                     // standard servos
    Servo grabRight;


    //servo to score blocks in the goal
    Servo hopper;                          //CR servo

    //servo for autonomous for the climbers
    Servo shelterDrop;                   //CR servo IMPORTANT THIS HAS NOT BEEN ADDED YET

    //servo to activate the hanger grabbers
    Servo hangArms;                     //CR servo

    //servo for debris sweeping away
    Servo windowWiper;                  //Standard Servo

    //servo for hitting the all clear
    Servo allClear;                     //Standard Servo

    Servo ziplineBar;                   //Micro Servo

    Servo hookPoles;                    //CR Servo

    //variables for driving
    double trailingPowerRight;                  //this code allows us to always give slightly
    double leadingPowerRight;                   //less power to leading motor to always
    double trailingPowerLeft;                   //ensure tension between the treads and
    double leadingPowerLeft;                    //ground for maximum driver control
    double m = 1.0;                             //slope for the gain
    double b = 0.3;                             //y intercept for the gain
    //servo variables for all clear servo

    double allClearUp = 0.0;
    double allClearDown = 1.0;
    double allClearPosition = allClearDown;

    //servo variables for grab servos
    double grabLeftUp = 0.0;                //0 is max CCW (UP on left side)
    double grabLeftDown = 0.6;              //0.6 is approx. 90 degrees CW (DOWN on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double grabRightDown = 0.4;             //0.4 is approx. 90 degrees CCW (DOWN on right side)

    //servo variables for the hanging servo
    double hangOut = 1.0;
    double hangIn = 0.0;
    double hangPosition = hangIn;

    //servo variable for continuous rotation hopper servo
    double runHopperLeft = 1.0;             //according to the google doc a value of 255 scores when on red.
    double runHopperRight = 0.0;            //running these servos at full speed (CR servos)
    double hopperStopMovingRed = 0.51;      //this value is at 0.51 because the servo on the original hopper
    double hopperStopMovingBlue = 0.49;
    double hopperStopMoving = 0.5;          //this value is only used in init
    double hopperPower = 0.51;              //was not stopping entirely at 0.5 value

    //servo variables to place the climber into the dump area behind the beacon in auto
    double shelterDropScored = 0.0;
    double shelterDropRetracted = 1.0;
    double shelterDropPosition = shelterDropRetracted;

    //servo variables for the window wiper to sweep away blocks from the ramp
    double windowWiperOpened = 0.75;        //THIS VALUE FOR OPENING WAS ON THE ORIGINAL TELEOP ~NEEDS TESTING
    double windowWiperClosed = 0;
    double windowWiperPosition = windowWiperClosed;


    //gives the state of the magnet sensors for the LED activation and ability to stop the motors
    boolean armsNotExtended = true;    // state of magnetic sensors to false to light up
    boolean armsNotRetracted = true;   //for initialization sequence

    //variables for the winch motors to allow automatic control with manual override
    double leftWinchPower = 0;
    double rightWinchPower = 0;

    //These joysticks are the initial gains that are assigned to the driver control
    double joystickGainR = 1;
    double joystickGainL = 1;

    float joystick1ValueRight;
    float joystick1ValueLeft;

    float joystick2ValueRight;
    double joystick2GainRight = 1;

    //these values are used later on for the hopper servo when collecting and spinning
    //red and blue are the only two possibilities
    //This function COULD be made to use one variable but we use two for READING purposes
    boolean redTeam = true;
    boolean blueTeam = false;

    @Override
    public void init() {

        //NAMES FOR CONFIGURATION FILES ON ZTE PHONES

        //THIS SHOULD ALL BE REORGANIZED IN ORDER OF IMPORTANCE

        //gives name of magnetic sensors
        //These magnetic sensors are placed on the arms of the robot which deploy the blocks into the
        //bucket. They return true (1) when no magnet detected nearby false (0) when magnet detected
        //These values get input into the variables armsNotExtended and armsNotRetracted
        //These values are used for the LED indicators and, more importantly, when extending the arms.
        //If the arms are not stopped correctly they have the potential to fly off the robot with their momentum.
        //Adding the magnetic sensors to the conditional for extending the arms deals with that risk.
        sensorExtend = hardwareMap.digitalChannel.get("mag1");
        sensorRetract = hardwareMap.digitalChannel.get("mag2");

        //configuration of the LED indicators
        //These LEDs are placed on the outside of the robot in the view of the drivers so that they accurately know
        //when the arms are fully extended or retracted.
        extendLED = hardwareMap.digitalChannel.get("led1");
        retractLED = hardwareMap.digitalChannel.get("led2");
        extendLED.setMode(DigitalChannelController.Mode.OUTPUT);        //the LEDs will be given a logical
        retractLED.setMode(DigitalChannelController.Mode.OUTPUT);       //output signal to turn on/off
        retractLED.setState(false);                                     //LEDs are initialized to (0) or "on"
        extendLED.setState(false);                                      //to make sure that they are functioning


        //gives name of drive motors
        driveLeftBack = hardwareMap.dcMotor.get("m5");              // 1 on red controller SN VUTK
        driveLeftFront = hardwareMap.dcMotor.get("m6");             // 2 on red

        driveRightBack = hardwareMap.dcMotor.get("m1");             // 1 on purple controller SN UVQF
        driveRightFront = hardwareMap.dcMotor.get("m2");            // 2 on purple

        // set direction of L and R drive motors, since they are opposite-facing

        driveRightFront.setDirection(DcMotor.Direction.FORWARD);    // LEFT side forward
        driveRightBack.setDirection(DcMotor.Direction.FORWARD);     // with positive voltage
        driveLeftBack.setDirection(DcMotor.Direction.REVERSE);      // so we reverse the RIGHT side
        driveLeftFront.setDirection(DcMotor.Direction.REVERSE);     //THIS CONFIGURATION NEEDS TO BE CHECKED

        //The encoders on the robot are present on an idler wheel within the tread to get the actual distance
        //and not simply the amount of rotations on the motor. The motor clicks can not be used to calculate
        //a true distance.
        //Other encoders are found on the actual motors on two of the motors within the tread.
        //Encoders are not currently used during TeleOp operation

        //gives motor names for the other motors
        arms = hardwareMap.dcMotor.get("m7");                       // 1 on green controller SN VF7F
        spin = hardwareMap.dcMotor.get("m8");                       // 2 on green

        //gives names of winch motors in the configuration files
        leftWinch = hardwareMap.dcMotor.get("m4");                  // 1 on orange controller SN XTJI (retract[+])
        rightWinch = hardwareMap.dcMotor.get("m3");                 // 2 on orange (extend [-])

        //give the servo names for the servos
        grabLeft = hardwareMap.servo.get("s1");
        grabRight = hardwareMap.servo.get("s2");
        hookPoles = hardwareMap.servo.get("s3");
        hopper = hardwareMap.servo.get("s4");
        windowWiper = hardwareMap.servo.get("s5");
        shelterDrop = hardwareMap.servo.get("s6");
        allClear = hardwareMap.servo.get("s7");
        ziplineBar = hardwareMap.servo.get("s8");

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        windowWiper.setPosition(windowWiperClosed);
        hopper.setPosition(hopperStopMoving);
        hangArms.setPosition(hangIn);
        shelterDrop.setPosition(shelterDropPosition);
        allClear.setPosition(allClearPosition);

        this.resetStartTime();     //reset to allow time for servos to reach initialized positions

        while (this.getRuntime() < 1) {

        }

        //reset timer for match
        this.resetStartTime();

    }

    @Override
    public void loop() {

            //

            /*
                SETTING MAGNETIC SENSORS
            */

            //creates boolean value for magnetic sensor,
            //true (1)= no magnet detected nearby
            //false (0) = magnet detected -> ARM HAS REACHED LIMIT
            armsNotExtended = sensorExtend.getState();
            armsNotRetracted = sensorRetract.getState();
            /*
                SETTING LED STATES
            */

            if (armsNotRetracted) {                 //set states of LED based on the positions of
                retractLED.setState(false);         //the magnet sensor and magnet
            } else {
                retractLED.setState(true);
            }
            if (armsNotExtended) {
                extendLED.setState(false);
            } else {
                extendLED.setState(true);
            }

            /*
                ASSIGNING RED OR BLUE TEAM
            */


            //The driver can assign either red or blue by clicking the Red (B) button or the
            // Blue button (X)
            //This will be done in TeleOp because :
            //  1. I dont know if it is possible in init
            //  2. It cannot be reversed if it is only in init
            if (gamepad2.x) {

                redTeam = false;
                blueTeam = true;

            } else if (gamepad2.b) {

                blueTeam = false;
                redTeam = true;

            }

            /* ALL DRIVING IS CALLED HERE. GAMEPAD VALUES ARE PASSED AS PARAMETERS */

            if (((Math.abs(gamepad1.right_stick_y))>0.1)||((Math.abs(gamepad1.left_stick_y))>0.1)){

                TeleOpDrive(gamepad1.right_stick_y, gamepad1.left_stick_y);

            }

            /*
                  SPINNER AND HOPPER FUNCTIONALITY
            */

            /*This program structure sets the spin and hopper functions in the same else loop which
            means that the drivers cannot control each function individually. CHANGE THIS.*/

            if (gamepad2.a) {
                spin.setPower(-1);              //a negative value to the spin collects debris

                //here if we are on the blue team or on the red team we need the collector box to work in different
                //directions. This is a good place to use red and blue team values. I know it can be done with one variable.
                if (blueTeam) {
                    hopperPower = runHopperLeft;
                    //On the blue team, collection is to the left
                } else {
                    hopperPower = runHopperRight;
                    //On the red team, collection is to the right
                }

            } else if (gamepad2.y) {

                spin.setPower(1);

                if (blueTeam) {
                    hopperPower = hopperStopMovingBlue;
                } else {
                    hopperPower = hopperStopMovingRed;
                }

                //when the spinner is ejecting stuff we don't need the hopper to be collecting
                //items, so we set the servo to stop moving.

            } else if (gamepad1.x) {

                //this is just hopper functionality:
                hopperPower = runHopperLeft;
                spin.setPower(0);

            } else if (gamepad1.b) {

                hopperPower = runHopperRight;
                spin.setPower(0);

            } else {

                spin.setPower(0);

                if (blueTeam) {
                    hopperPower = hopperStopMovingBlue;
                } else {
                    hopperPower = hopperStopMovingRed;
                }
            }

            hopper.setPosition(hopperPower);

            /*
                WINDOW WIPER (SWEEP AWAY BLOCKS)
            */

            if (gamepad1.a) {
                windowWiperPosition = windowWiperOpened;
            } else {
                windowWiperPosition = windowWiperClosed;
            }

            windowWiper.setPosition(windowWiperPosition);

              /*
                CHURRO GRABBERS ARE SET
              */

            //takes input from bumpers and triggers for the locking grab motors set individually
            if (gamepad1.right_bumper) {
                grabRight.setPosition(grabRightUp);
            } else if (gamepad1.right_trigger > .3) {       //these triggers have full 0-1 ranges, but we use them as
                grabRight.setPosition(grabRightDown);       //plain buttons by applying a threshold
            }


            if (gamepad1.left_bumper) {
                grabLeft.setPosition(grabLeftUp);
            } else if (gamepad1.left_trigger > .3) {
                grabLeft.setPosition(grabLeftDown);
            }

            /*
                    HANGING SERVO
             */

            //this conditional allows the hanging servos to deploy only in the last minute and a half of the game
            //whn both of the gunners triggers are pressed fully. They can always be brought in using the bumpers
            if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3 && this.getRuntime() > 90)
            {
                hangPosition = hangOut;
            }
            else if (gamepad2.left_bumper && gamepad2.right_bumper){
                hangPosition = hangIn;
            }
            hangArms.setPosition(hangPosition);

            /*
                    ALL ClEAR SERVO
             */

            if (gamepad2.dpad_right){
                allClearPosition = allClearUp;
            } else if (gamepad2.dpad_left) {
                allClearPosition = allClearDown;
            }
            allClear.setPosition(allClearPosition);

            /*
                ARM EXTENSION
             */

            if ((gamepad2.dpad_up || gamepad1.dpad_up) && armsNotExtended) {      //moves arms with d-pad buttons, add time phased power

                arms.setPower(-1);

            } else if ((gamepad2.dpad_down || gamepad1.dpad_down) && armsNotRetracted) {

                arms.setPower(1);
            } else {
                arms.setPower(0);
            }

            if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {    //allow manual
                leftWinchPower = gamepad2.left_stick_y;                         //override of the winch
            }                                                                   //motors when driver
            if (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1) {  //wants control
                rightWinchPower = gamepad2.right_stick_y;
            } else {
                leftWinchPower = 0;
                rightWinchPower = 0;
            }

            rightWinch.setPower(rightWinchPower);           //sets power of the winches to the
            leftWinch.setPower(leftWinchPower);             //specified power

    }//finish loop

    /*DRIVING FUNCTION WITH CONTINUOUS GAIN AND LEADING & TRAILING VALUES*/
    public void TeleOpDrive(float joystick1ValueRight, float joystick1ValueLeft) {

        //JUST USE ONE VARIBLE

        //takes input from joysticks for motor values;
        // sets the front wheel at a lesser power to ensure belt tension
        //If the driver moves the joystick upward the motors get a negative power.

        if ((((Math.abs(joystick1ValueLeft)) >= 0.1)) && (((Math.abs(joystick1ValueLeft)) <= 0.7))) {
            joystickGainL = m * joystick1ValueLeft + b;
        }

        if ((((Math.abs(joystick1ValueRight)) >= 0.1)) && (((Math.abs(joystick1ValueRight)) <= 0.7))) {
            joystickGainR = m * joystick1ValueRight + b;
        }

        trailingPowerLeft = joystick1ValueLeft * joystickGainL;
        leadingPowerLeft = .95 * trailingPowerLeft;
        trailingPowerRight = joystick1ValueRight * joystickGainR;
        leadingPowerRight = .95 * trailingPowerRight;

        /*
            SETTING LEFT POWERS AND DEADZONES
        */

        //Here we assign the leading powers to the motor that
        //is in front according to the direction of the robot
        //and the trailing motor value to the back. This is determined
        //using the values of the joystick. The dead zone is between
        //-0.1 and 0.1

        if (leadingPowerLeft > 0.1) {                         //left treads driving forward
            driveLeftBack.setPower(trailingPowerLeft);
            driveLeftFront.setPower(leadingPowerLeft);
        } else if (leadingPowerLeft < -0.1) {                 //left tread moving backward
            driveLeftBack.setPower(leadingPowerLeft);
            driveLeftFront.setPower(trailingPowerLeft);
        } else {                                              //ignore very low powers
            driveLeftFront.setPower(0);
            driveLeftBack.setPower(0);
        }


        /*
            SETTING RIGHT POWERS AND DEADZONES
        */

        if (leadingPowerRight > 0.1) {                      //right tread moving forward
            driveRightBack.setPower(trailingPowerRight);
            driveRightFront.setPower(leadingPowerRight);
        } else if (leadingPowerRight < -0.1) {              //right tread moving backward
            driveRightBack.setPower(leadingPowerRight);
            driveRightFront.setPower(trailingPowerRight);
        } else {                                            //ignore very low values
            driveRightBack.setPower(0);
            driveRightFront.setPower(0);
        }
    }

    public void ArmsForGunner(float joystick2ValueRight){

        if ((((Math.abs(joystick2ValueRight)) >= 0.1)) && (((Math.abs(joystick2ValueRight)) <= 0.7))) {
            joystick2GainRight = m * joystick2ValueRight + b;
        }
    }

}//finish program