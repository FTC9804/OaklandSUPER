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
 * v1 3-5-16 at 5:37 pm Steve -- test code with additional servos to drive for set distance; corrected the effective dia of wheel
 * v2 3-5-16 at 8:22 pm Steve -- test code for near->near red
 * v3 3-12-16 at 8:00 pm Steve -- test code for near->near red with methods
 * v4 3-15-16 at 8:06 pm Steve -- test code with methods mapped out
 * v5 3-20-16 at 11:40 pm Steve -- test code with updated methods based on Google Drive Notes
 *
 *
 * SetUp:
 * Back left edge of first full box from the mountain on the red side
 * Facing the shelter BACKWARDS
 *
 * Movement:
 * Drive for 2*sqrt(2)*12 = 33.94 inches backwards with spin motors running
 * Spins CW 90º
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


public class Oak_9804_RED_Auto_NearNear_v5 extends LinearOpMode {

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
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
        //2 encoders are on rear motors, two are mounted in tread module

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


        driveStraightBackwards(0, 33.94, 0.5); //the distance is absolute, the heading is incremental, the mid power is absolute

        stopMotors();

        spinMoveClockwise(-90); //the heading is incremental

        stopMotors();

        windowWiperActivate();

        driveStraightForwards(-90, 24, 0.5); //the distance is absolute, the heading is incremental, the mid power is absolute

        stopMotors();

        objectiveAttained();

    }//finish the opmode

    void stopMotors (){
        //set all motor powers to 0 after some code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower(0);

        telemetry.addData("STOP MOTORS", telemetryVariable);
    }


    void objectiveAttained (){
        stopMotors();
        telemetry.addData("CODE COMPLETE", telemetryVariable);
    }

    void windowWiperActivate (){
        //CLEAR DEBRIS WITH WINDOW WIPER
        windowWiper.setPosition(sweepOpened);
        this.resetStartTime();
        while (this.getRuntime() < 1){
        }
        windowWiper.setPosition(sweepClosed);
        this.resetStartTime();
        while (this.getRuntime() < 0.5) {
        }

    }

    void spinMoveCounterClockwise (int heading){

        //SPIN MOVE
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        driveGain = 0.05;       //OK for spin
        targetHeading = heading;    //90º CCW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            currentHeading = gyro.getIntegratedZValue();

            telemetry.addData("current signed heading: ", currentHeading);

            headingError = targetHeading - currentHeading;//for CCW spin from 0 to 90º, error always positive

            driveSteering = headingError*driveGain;         //positive value

            //for CCW spin, left tread forward
            leftPower = - driveSteering;
            if(leftPower < -1){
                leftPower = -1 ;
            }
            if (leftPower > -0.2){           //avoid zero closing power at low error
                leftPower = -0.2;            //0.1 stalled near target heading
            }


            //for CCW spin, right tread runs forwards
            rightPower = driveSteering;
            if (rightPower > 1){
                rightPower = 1;
            }
            if (rightPower < 0.2){
                rightPower = 0.2;
            }

            //when spinning CCW, left front is trailing, left back is leading
            //right front is leading, right back is trailing
            //trailing gets full power

            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(0.95 * leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

        } while (currentHeading > targetHeading
                && this.getRuntime() < 6);
        //spin from 0 to - number, so loop while 'greater than' the target heading

        telemetry.addData("SPIN CCW DONE", telemetryVariable);

    }

    void spinMoveClockwise (int heading) {

        //SPIN MOVE
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        driveGain = 0.05;       //OK for spin
        targetHeading = heading;    //90º CW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            currentHeading = gyro.getIntegratedZValue();

            telemetry.addData("current signed heading: ", currentHeading);

            headingError = targetHeading - currentHeading;//for CW spin from 0 to -90º, error always negative

            driveSteering = headingError*driveGain;         //negative value

            //for CW spin, left tread runs forwards
            leftPower = - driveSteering;
            if(leftPower > 1){
                leftPower = 1 ;
            }
            if (leftPower < 0.2){           //avoid zero closing power at low error
                leftPower = 0.2;            //0.1 stalled near target heading
            }


            //for CW spin, right tread runs backwards
            rightPower = driveSteering;
            if (rightPower < -1){
                rightPower = -1;
            }
            if (rightPower > -0.2){
                rightPower = -0.2;
            }

            //when spinning CW, left front is leading, left back is trailing
            //right front is trailing, right back is leading
            //trailing gets full power
            driveLeftFront.setPower(0.95 * leftPower);
            driveLeftBack.setPower(leftPower);
            driveRightFront.setPower(rightPower);
            driveRightBack.setPower(0.95 * rightPower);

        } while (currentHeading < targetHeading
                && this.getRuntime() < 6);
        //spin from 0 to + number, so loop while 'less than' the target heading

        telemetry.addData("SPIN CW DONE", telemetryVariable);

    }

    void driveStraightForwards (int heading, double distance, double midPower){      //first one is 90,24

        telemetry.clearData();
        //DRIVE FORWARD 24 INCHES
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        driveGain = 0.05;           //gain used for proportional steering
        targetHeading = heading;        //drive straight ahead, same heading

        targetDistance = distance;
        rotations = targetDistance/circumference;
        targetEncoderCounts = (int)(encoderCountsPerRotation*rotations);

        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();

        this.resetStartTime();      //for safety timeout

        //loop until driving distance reached (or safety timeout)
        do {
            currentEncDeltaCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;
            currentEncDeltaCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;

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

        } while (EncErrorLeft > 0 && this.getRuntime() < 12);


        telemetry.addData("STRAIGHT FORWARDS DONE", telemetryVariable);

    }

    void driveStraightBackwards (int heading, double distance, double midPower) {

        //DRIVE BACKWARDS DESIRED INCHES

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        targetHeading = heading;              //drive straight ahead at the initial/default heading

        targetDistance = distance;          //drive straight 50.9117 inches

        //math for target encoder counts to travel
        rotations = targetDistance / circumference;
        targetEncoderCounts = (int) (encoderCountsPerRotation * rotations);

        spin.setPower(0);

        //resets start time before starting the drive
        this.resetStartTime();

        //takes the initial position of the encoders to establish a starting point for the distance
        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();


        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            currentEncDeltaCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;         //the current - initial will give the
            currentEncDeltaCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;      //current distance of the encoders


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



        } while (EncErrorLeft > 0                   //the error is slowly decreasing, so run while greater than 0
                && this.getRuntime() < 12);         //safety timeout of 100 seconds


        telemetry.addData("DRIVE STRAIGHT BACKWARDS DONE", telemetryVariable);

    }

}//finish the code
