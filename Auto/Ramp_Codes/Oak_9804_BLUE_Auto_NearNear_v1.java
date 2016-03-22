package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * FTC Team 9804 Bomb Squad Autonomous
 * Made by the programmers of FTC Team 9804 Bomb Squad
 *
 * Drives a predetermined set distance
 *
 * v1 3-12-16 at 7:01 pm Steve -- test code for near->near blue
 *
 *
 * SetUp:
 * Back right edge of first full box from the mountain on the blue side
 * Facing the shelter BACKWARDS
 *
 * Movement:
 * Drive for 2*sqrt(2)*12 = 33.94 inches backwards with spin motors running
 * Spins CCW 90º
 * window wiper servo
 * drive FORWARDS 24 inches
 *
 *
 * GENERAL RULE:
 *  FWD: leftPower = midPower - driveSteering;
 *  BWD: leftPower = midPower + drive Steering
 *  CCW: positive
 *  CW: negative
 */


public class Oak_9804_BLUE_Auto_NearNear_v1 extends LinearOpMode {

    //drive motors
    DcMotor driveLeftBack;
    DcMotor driveLeftFront;
    DcMotor driveRightBack;
    DcMotor driveRightFront;
    //two encoders are on rear motors, 2 are mounted on tread module
    DcMotor spin;

    //servos to lock in place on ramp
    Servo grabLeft;
    Servo grabRight;

    //extra servo
    Servo box;

    //servo to push away debris from ramp
    Servo windowWiper;

    double midPower;
    int targetHeading;
    double driveGain;
    double leftPower;
    double rightPower;
    int currentHeading = 0;                     //This is a signed value, CCW is positive
    int headingError;
    double driveSteering;
    double currentDistance;
    int currentEncDeltaCountLeft;
    int currentEncDeltaCountRight;


    double targetDistance;                      //magnitude in distance
    final int encoderCountsPerRotation = 1120;
    final double diameter = 2.583;       //effective diameter of drive pulley and tread b/c of sinking into foam mat (measures 3" dia)
    double circumference = diameter * 3.14159; //C=PI*D
    double rotations;
    int targetEncoderCounts;
    int EncErrorLeft;
    int telemetryVariable;
    int initialEncCountLeft;
    int initialEncCountRight;


    //servo variables
    double grabLeftUp = 0;                  //0 is max CCW (UP on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double sweepOpened = 0.75;
    double sweepClosed = 0;
    double sweepPosition = sweepClosed;
    double boxPosition = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        //USE CONFIGURATION FILE 'JABBED' ON BOTH MAIN AND B PHONES


        //gives name of drive motors
        driveLeftBack = hardwareMap.dcMotor.get("m5");      // 1 on red controller SN VUTK
        driveLeftFront = hardwareMap.dcMotor.get("m6");     // 2 on red
        driveRightBack = hardwareMap.dcMotor.get("m1");     // 1 on purple controller SN UVQF
        driveRightFront = hardwareMap.dcMotor.get("m2");    // 2 on purple
        spin = hardwareMap.dcMotor.get("m8");
        //give the servo names for the servos
        grabLeft = hardwareMap.servo.get("s1");             // xx on servo controller SN VSI1
        grabRight = hardwareMap.servo.get("s2");            // xx on servo controller

        windowWiper = hardwareMap.servo.get("s5");

        box = hardwareMap.servo.get("s4");

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        windowWiper.setPosition(sweepPosition);
        box.setPosition(boxPosition);


        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        hardwareMap.logDevices();

        gyro.calibrate();


        //begin match with driver station button
        waitForStart();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
        //2 encoders are on rear motors, two are mounted in tread module


        //DRIVE BACKWARDS 33.94 INCHES

        telemetry.clearData();      //clear all telemtry data before starting

        driveGain = 0.05;           //gain for proportional control

        midPower = 0.5;             //the midpower with which we add and subtract the drive steering from
        targetHeading = 0;              //drive straight ahead at the initial/default heading

        targetDistance = 33.94;          //drive straight 33.94 inches

        //math for target encoder counts to travel
        rotations = targetDistance / circumference;
        targetEncoderCounts = (int) (encoderCountsPerRotation * rotations);

        spin.setPower(0);

        //resets start time before starting the drive
        this.resetStartTime();

        //takes the initial position of the encoders to establish a starting point for the distance
        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();

        //allow time to read encoder value
        for (int i=0; i< 10; i++){
            waitOneFullHardwareCycle();
        }


        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            currentEncDeltaCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;         //the current - initial will give the
            currentEncDeltaCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;      //current distance of the encoders

            waitOneFullHardwareCycle();         //allow time for the hardware to execute its task

            EncErrorLeft = targetEncoderCounts - Math.abs(currentEncDeltaCountLeft);                     //the error is the delta between the target counts and current counts


            //telemetry for encoder information
            telemetry.addData("EncErrorLeft = ", EncErrorLeft);
            telemetry.addData("Left Encoder: ", currentEncDeltaCountLeft);

            //telemetry for the distance travelled (IN INCHES)
            currentDistance = (currentEncDeltaCountLeft * circumference) / encoderCountsPerRotation;
            telemetry.addData("Calculated current distance: ", currentDistance);

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGain;       //create the proportion for the steering

            leftPower = midPower + driveSteering;           //adds the drive steering to midpower because we are driving backwards
            if (leftPower > 1.0) {                            //cuts ourselves off at 1, the maximum motor power
                leftPower = 1.0;
            }
            if (leftPower < 0.2) {                            //CHANGE THIS TO A CORRECTLY TESTED NUMBER
                leftPower = 0.2;                              //WE WANT THE LOWEST POWER THE ROBOT CAN DRIVE AT
            }
            rightPower = midPower - driveSteering;          //subtraction because we are driving backwards
            if (rightPower > 1.0) {
                rightPower = 1.0;
            }
            if (rightPower < 0.2) {
                rightPower = 0.2;
            }
            //when driving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(-leftPower);
            driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(-rightPower);
            driveRightBack.setPower(-.95 * rightPower);

            waitOneFullHardwareCycle();             //gives the code time to run


        } while (EncErrorLeft > 0                   //the error is slowly decreasing, so run while greater than 0
                && this.getRuntime() < 100);         //safety timeout of 100 seconds


        telemetry.addData("DRIVE STRAIGHT DONE", telemetryVariable);

        //set all motor powers to 0 after drive code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower (0);

        this.resetStartTime();
        while(this.getRuntime() < 10){
            waitOneFullHardwareCycle();
        }

        //SPIN CCW 90º
        midPower = 0;           //spin move, zero driving-forward power
        driveGain = 0.05;       //OK for spin
        targetHeading = 90;    //90º CCW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            currentHeading = gyro.getIntegratedZValue();

            telemetry.addData("current signed heading: ", currentHeading);

            headingError = targetHeading - currentHeading;//for CCW spin from 0 to 90º, error always positive

            driveSteering = headingError*driveGain;         //positive value

            //for CCW spin, left tread runs backwards
            leftPower = midPower - driveSteering;
            if(leftPower > -1){
                leftPower = -1 ;
            }
            if (leftPower < -0.2){           //avoid zero closing power at low error
                leftPower = -0.2;            //0.1 stalled near target heading
            }


            //for CCW spin, right tread runs forwards
            rightPower = midPower + driveSteering;
            if (rightPower < 1){
                rightPower = 1;
            }
            if (rightPower > 0.2){
                rightPower = 0.2;
            }

            //when spinning CCW, left front is trailing, left back is leading
            //right front is leading, right back is trailing
            //trailing gets full power
            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(0.95 * leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

            waitOneFullHardwareCycle();

        } while (currentHeading < targetHeading && this.getRuntime() < 60);
        //spin from 0 to 90, so loop while 'less than' the target heading

        telemetry.addData("SPIN MOVE 90 CCW DONE", telemetryVariable);

        //set all motor powers to 0 after drive code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower(0);

        this.resetStartTime();
        while(this.getRuntime() < 10){
            waitOneFullHardwareCycle();
        }


        //CLEAR DEBRIS WITH WINDOW WIPER
        windowWiper.setPosition(sweepOpened);
        this.resetStartTime();
        while (this.getRuntime() < 1){
            waitOneFullHardwareCycle();
        }
        windowWiper.setPosition(sweepClosed);
        this.resetStartTime();
        while (this.getRuntime() < 0.5) {
            waitOneFullHardwareCycle();
        }


        //DRIVE FORWARD 24 INCHES
        telemetry.clearData();

        midPower = 0.66;            //default value for driving, adjusted by steering
        driveGain = 0.05;           //gain used for proportional steering
        targetHeading = 90;        //drive straight ahead, same heading

        targetDistance = 24;
        rotations = targetDistance/circumference;
        targetEncoderCounts = (int)(encoderCountsPerRotation*rotations);

        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();

        //allow time to read encoder value
        for (int i = 0; i < 10; i++) {
            waitOneFullHardwareCycle();
        }

        this.resetStartTime();      //for safety timeout

        //loop until driving distance reached (or safety timeout)
        do {
            currentEncDeltaCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;
            currentEncDeltaCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;
            waitOneFullHardwareCycle();

            EncErrorLeft = targetEncoderCounts - Math.abs(currentEncDeltaCountLeft);

            telemetry.addData("Left Encoder Delta:", currentEncDeltaCountLeft);

            currentDistance = (currentEncDeltaCountLeft/encoderCountsPerRotation) * circumference;

            telemetry.addData("Calculated current distance: ", currentDistance);

            currentHeading = gyro.getIntegratedZValue();

            telemetry.addData("current signed heading: ", currentHeading);

            headingError = targetHeading - currentHeading;      //positive if pointing too far CW

            driveSteering = headingError * driveGain;           //positive if pointing too far CW

            leftPower = midPower - driveSteering;
            if (leftPower > 1) {
                leftPower = 1;
            }
            if (leftPower < 0.2) {
                leftPower = 0.2;
            }


            rightPower = midPower + driveSteering;
            if (rightPower > 1) {
                rightPower = 1;
            }
            if (rightPower < 0.2) {
                rightPower = 0.2;
            }

            //when driving forward, left front is leading, left back is now trailing, same for right
            //trailing gets full power
            driveLeftFront.setPower(0.95 * leftPower);
            driveLeftBack.setPower(leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

            waitOneFullHardwareCycle();

        } while (EncErrorLeft > 0 && this.getRuntime() < 8);


        telemetry.addData("STRAIGHT 2 DONE", telemetryVariable);

        //set all motor powers to 0 after drive code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower(0);


        //telemetry to display that the code has finished
        telemetry.addData("CODE COMPLETE", telemetryVariable);


    }//finish the opmode
}//finish the code
