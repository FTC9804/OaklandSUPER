package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/* Made by the programmers of FTC Team 9804 Bomb Squad
 *v1 3-5-16 at 5:37 pm Steve -- test code with additional servos to drive for set distance
 */


public class Oak_9804_Auto_v1 extends LinearOpMode {

    //drive motors
    DcMotor driveLeftBack;
    DcMotor driveLeftFront;
    DcMotor driveRightBack;
    DcMotor driveRightFront;
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
    double driveGain = 0.1;
    double leftPower;
    double rightPower;
    int currentHeading = 0;                     //This is a signed value
    int headingError;
    double driveSteering;
    double currentDistance;
    int currentEncCountLeft;
    int currentEncCountRight;


    double targetDistance;
    int encoderCountsPerRotation = 1120;
    double radius = 1.86; //using math is 1.86
    double diameter = radius * 2;
    double circumference = diameter * 3.14159;
    double rotations;
    int targetEncoderCounts;
    int targetEncoderCounts1;
    int EncErrorLeft;
    int telemetryVariable;
    int initialEncCountLeft;
    int initialEncCountRight;


    //servo variables
    double grabLeftUp = 0;                  //0 is max CCW (UP on left side)
    double grabLeftDown = 0.6;              //0.6 is approx. 90 degrees CW (DOWN on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double grabRightDown = 0.4;             //0.4 is approx. 90 degrees CCW (DOWN on right side)
    double sweepOpened = 0.75;
    double sweepClosed = 0;
    double sweepPosition = sweepClosed;
    double boxPosition = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);



        //DRIVE BACKWARDS 60 INCHES

        telemetry.clearData();      //clear all telemtry data before starting

        driveGain = 0.05;           //gain for proportional control

        midPower = 0.5;             //the midpower with which we add and subtract the drive steering from
        targetHeading = 0;              //drive straight ahead at the initial heading

        targetDistance = 60;          //drive straight 60 inches

        //math for target encoder counts to travel
        rotations = targetDistance / circumference;
        targetEncoderCounts = (int) (encoderCountsPerRotation * rotations);
        targetEncoderCounts1 = targetEncoderCounts;

        //resets start time before starting the drive
        this.resetStartTime();

        //takes the initial position of the encoders to establish a starting point for the distance
        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();


        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            currentEncCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;         //the current - initial will give the
            currentEncCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;      //current distance of the encoders

            EncErrorLeft = targetEncoderCounts - Math.abs(currentEncCountLeft);                     //the error is the delta between the target counts and current counts


            //telemetry for encoder information
            telemetry.addData("EncErrorLeft = ", EncErrorLeft);
            telemetry.addData("Left Encoder: ", currentEncCountLeft);

            //telemetry for the distance travelled (IN INCHES)
            currentDistance = (currentEncCountLeft * circumference) / encoderCountsPerRotation;
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
                && this.getRuntime() < 30);         //safety timeout of 10 seconds

        //set all motor powers to 0 after drive code finished running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower (0);

        //telemetry to display that the code has finished
        telemetry.addData("CODE COMPLETE", telemetryVariable);


    }//finish the opmode
}//finish the code
