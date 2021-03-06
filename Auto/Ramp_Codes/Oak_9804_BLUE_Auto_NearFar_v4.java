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
 * v1 3-12-16 at 7:21 pm Steve -- test code for far->near blue
 * v2 3-12-16 at 8:03 pm Steve -- test code for near->near blue with methods
 * v3 3-15-16 at 6:36 pm Steve -- test code with methods set up in competition layout
 * v4 3-15-16 at 8:06 pm Steve -- test code with methods mapped out
 *
 * SetUp:
 * Back left edge of second full box from the mountain on the blue side
 * Facing the shelter BACKWARDS
 *
 * Movement:
 * Drive for 1.5*2*sqrt(2)*12 = 50.9117 inches backwards with spin motors running
 * Spins CCW 90º
 * drive FORWARDS 24 inches
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


public class Oak_9804_BLUE_Auto_NearFar_v4 extends LinearOpMode {

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


        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        hardwareMap.logDevices();

        gyro.calibrate();

        configurationSetup();

        //begin match with driver station button
        waitForStart();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        motorOrientationEstablishment();

        driveStraightBackwards(0, 50.9117);

        stopMotors();

        spinMoveCounterClockwise(90);

        stopMotors();

        driveStraightForwards(90, 24);

        stopMotors();

        windowWiperActivate();

        stopMotors();

        driveStraightForwards(90, 24);

        stopMotors();

        objectiveAttained();


    }//finish the opmode

    void configurationSetup () {


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

    }

    void stopMotors (){
        //set all motor powers to 0 after some code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower(0);

        telemetry.addData("STOP MOTORS", telemetryVariable);
    }

    void delayActivation (){

        //DELAY 15 SECONDS
        stopMotors();

        //allow time to read encoder value
        for (int i = 0; i < 10; i++) {
        }

    }

    void motorOrientationEstablishment () {
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
        //2 encoders are on rear motors, two are mounted in tread module

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

        midPower = 0;           //spin move, zero driving-forward power
        driveGain = 0.05;       //OK for spin
        targetHeading = heading;    //90º CCW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            currentHeading = gyro.getIntegratedZValue();

            telemetry.addData("current signed heading: ", currentHeading);

            headingError = targetHeading - currentHeading;//for CCW spin from 0 to 90º, error always positive

            driveSteering = headingError*driveGain;         //positive value

            //for CCW spin, left tread runs backwards
            leftPower = midPower - driveSteering;
            if(leftPower < -1){
                leftPower = -1 ;
            }
            if (leftPower > -0.2){           //avoid zero closing power at low error
                leftPower = -0.2;            //0.1 stalled near target heading
            }


            //for CCW spin, right tread runs forwards
            rightPower = midPower + driveSteering;
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

        } while (currentHeading < targetHeading
                && this.getRuntime() < 60);
        //spin from 0 to + number, so loop while 'less than' the target heading

        telemetry.addData("SPIN 90 CCW DONE", telemetryVariable);

    }

    void driveStraightForwards (int heading, double distance){      //first one is 90,24

        telemetry.clearData();
        //DRIVE FORWARD 24 INCHES
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        midPower = 0.66;            //default value for driving, adjusted by steering
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

        } while (EncErrorLeft > 0 && this.getRuntime() < 8);


        telemetry.addData("STRAIGHT DONE", telemetryVariable);

    }

    void driveStraightBackwards (int heading, double distance) {

        //DRIVE BACKWARDS DESIRED INCHES

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        midPower = 0.5;             //the mid power with which we add and subtract the drive steering from
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
                && this.getRuntime() < 100);         //safety timeout of 100 seconds


        telemetry.addData("DRIVE STRAIGHT DONE", telemetryVariable);

        //set all motor powers to 0 after drive code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower(0);

    }

}//finish the code
