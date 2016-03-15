package com.qualcomm.ftcrobotcontroller.opmodes;

//import OpModes

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

/* Made by the programmers of FTC Team 9804 Bomb Squad
 *v1 3-2-16 at 7:58 pm Steve & Etienne -- created new Oakland tournament code with additional servos
 *v2 3-5-16 at 5:06 pm Etienne L. -added all clear servo
 */

public class Oak_9804_TeleOp_v1 extends OpMode {

    //defining the motors, servos, and variables in this program

    //magnetic sensor
    DigitalChannel sensorExtend;        // detects magnet, arms fully extended
    DigitalChannel sensorRetract;       // detects magnet, arms fully retracted
    DigitalChannel extendLED;           // these are indicator LEDs
    DigitalChannel retractLED;          // that will signal the drivers when arm limits are reached

    //drive motors
    DcMotor driveLeftBack;
    DcMotor driveLeftFront;
    DcMotor driveRightBack;
    DcMotor driveRightFront;

    //winch motors
    DcMotor leftWinch;
    DcMotor rightWinch;

    //motors for extending arms and spinner (debris collector)
    DcMotor arms;
    DcMotor spin;

    //servos to lock in place on ramp
    Servo grabLeft;
    Servo grabRight;


    //servo to score blocks in the goal
    Servo box;

    //servo for autonomous for the climbers
    Servo beaconDump;

    //servo to activate the hanger grabbers
    Servo hangArms;

    //servo for debris sweeping away
    Servo windowWiper;

    //servo for hitting the all clear
    Servo allClear;

    //variables for driving
    double trailingPowerRight;                //this code allows us to always give slightly
    double leadingPowerRight;                //less power to leading motor to always
    double trailingPowerLeft;                //ensure tension between the treads and
    double leadingPowerLeft;                //ground for maximum driver control

    //servo variables for all clear sevo

    double allClearUp = 0.0;
    double allClearDown = 1.0;
    double allClearPosition = allClearDown;

    //servo variables for grab servos
    double grabLeftUp = 0.0;                  //0 is max CCW (UP on left side)
    double grabLeftDown = 0.6;              //0.6 is approx. 90 degrees CW (DOWN on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double grabRightDown = 0.4;             //0.4 is approx. 90 degrees CCW (DOWN on right side)

    //servo variables for the hanging servo
    double hangOut = 1.0;
    double hangIn = 0.0;
    double hangPosition = hangIn;

    //servo variable for continuous rotation box servo
    double boxMovingDown = 0.8;
    double boxMovingUp = 0.2;
    double boxStopMoving = 0.51;
    double boxPower = 0.51;

    //servo variables to place the climber into the dump area behind the beacon in auto
    double beaconDumpScored = 0.0;
    double beaconDumpRetracted = 1.0;
    double beaconDumpPosition = beaconDumpRetracted;

    //servo variables for the window wiper to sweep away blocks from the ramp
    double windowWiperOpened = 0.75;
    double windowWiperClosed = 0;
    double windowWiperPosition = windowWiperClosed;


    //gives the state of the magnet sensors for the LED activation and ability to stop the motors
    boolean armsNotExtended = true;    // state of magnetic sensors to false to light up
    boolean armsNotRetracted = true;    //for initialization sequence

    //variables for the winch motors to allow automatic control with manual override
    double leftWinchSpeed = 0;
    double rightWinchSpeed = 0;

    double joystickGainR = 1;
    double joystickGainL = 1;

    double joystick1ValueRight;
    double joystick1ValueLeft;

    boolean redTeam = true;
    boolean blueTeam = false;

    @Override
    public void init() {

        //NAMES FOR CONFIGURATION FILES ON ZTE PHONES

        //gives name of magnetic sensors and LEDs for the configuration files
        sensorExtend = hardwareMap.digitalChannel.get("mag1");
        sensorRetract = hardwareMap.digitalChannel.get("mag2");
        extendLED = hardwareMap.digitalChannel.get("led1");
        retractLED = hardwareMap.digitalChannel.get("led2");
        extendLED.setMode(DigitalChannelController.Mode.OUTPUT);//the LEDs will be given a logical
        retractLED.setMode(DigitalChannelController.Mode.OUTPUT);//output signal to turn on/off
        retractLED.setState(false);                   // LEDs initialized "off"
        extendLED.setState(false);


        //gives name of drive motors
        driveLeftBack = hardwareMap.dcMotor.get("m5");      // 1 on red controller SN VUTK
        driveLeftFront = hardwareMap.dcMotor.get("m6");     // 2 on red

        driveRightBack = hardwareMap.dcMotor.get("m1");     // 1 on purple controller SN UVQF
        driveRightFront = hardwareMap.dcMotor.get("m2");    // 2 on purple

        // set direction of L and R drive motors, since they are opposite-facing
        driveRightFront.setDirection(DcMotor.Direction.FORWARD);  // LEFT side forward
        driveRightBack.setDirection(DcMotor.Direction.FORWARD);   // with positive voltage
        driveLeftBack.setDirection(DcMotor.Direction.REVERSE);    // so we reverse the RIGHT side
        driveLeftFront.setDirection(DcMotor.Direction.REVERSE);

        //gives motor names for the other motors
        arms = hardwareMap.dcMotor.get("m7");               // 1 on green controller SN VF7F
        spin = hardwareMap.dcMotor.get("m8");              // 2 on green

        //gives names of winch motors in the configuration files
        leftWinch = hardwareMap.dcMotor.get("m4");         // 1 on orange controller SN XTJI
        rightWinch = hardwareMap.dcMotor.get("m3");        // 2 on orange

        //give the servo names for the servos
        grabLeft = hardwareMap.servo.get("s1");             // xx on servo controller SN VSI1
        grabRight = hardwareMap.servo.get("s2");            // xx on servo controller
        hangArms = hardwareMap.servo.get("s3");
        box = hardwareMap.servo.get("s4");
        windowWiper = hardwareMap.servo.get("s5");
        beaconDump = hardwareMap.servo.get("s6");
        allClear = hardwareMap.servo.get("s7");

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        windowWiper.setPosition(windowWiperClosed);
        box.setPosition(boxStopMoving);
        hangArms.setPosition(hangIn);
        beaconDump.setPosition(beaconDumpPosition);
        allClear.setPosition(allClearPosition);

        this.resetStartTime();     //reset to allow time for servos to reach initialized positions

        while (this.getRuntime() < 1) {

        }

        //reset timer for match
        this.resetStartTime();

    }

    @Override
    public void loop() {
        //creates boolean value for magnetic sensor,
        //true (1)= no magnet detected nearby
        //false (0) = magnet detected
        armsNotExtended = sensorExtend.getState();
        armsNotRetracted = sensorRetract.getState();

        //takes input from joysticks for motor values;
        // sets the front wheel at a lesser power to ensure belt tension

        if ((gamepad1.back && gamepad1.x) || (gamepad2.back && gamepad2.x)) {

            redTeam = false;
            blueTeam = true;

        } else if ((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b)) {

            blueTeam = false;
            redTeam = true;

        }

        joystick1ValueLeft = gamepad1.left_stick_y;
        joystick1ValueRight = gamepad1.right_stick_y;

        if (Math.abs(joystick1ValueLeft) >= 0.1 && Math.abs(joystick1ValueLeft) < 0.4) {
            joystickGainL = 0.4;
        } else if (Math.abs(joystick1ValueLeft) >= 0.4 && Math.abs(joystick1ValueLeft) < 0.7) {
            joystickGainL = 0.7;
        } else {
            joystickGainL = 1;
        }
        if (Math.abs(joystick1ValueRight) >= 0.1 && Math.abs(joystick1ValueRight) < 0.4) {
            joystickGainR = 0.4;
        } else if (Math.abs(joystick1ValueRight) >= 0.4 && Math.abs(joystick1ValueRight) < 0.7) {
            joystickGainR = 0.7;
        } else {
            joystickGainR = 1;
        }

        trailingPowerLeft = joystick1ValueLeft * joystickGainL;
        leadingPowerLeft = .95 * trailingPowerLeft;
        trailingPowerRight = joystick1ValueRight * joystickGainR;
        leadingPowerRight = .95 * trailingPowerRight;

        if (leadingPowerLeft > 0.1) {                       // ignore dead zone on joystick
            driveLeftBack.setPower(trailingPowerLeft);      //dead zone is the area right next to 0,
            driveLeftFront.setPower(leadingPowerLeft);      //but not 0, where the motors are still
        } else if (leadingPowerLeft < -0.1) {               //straining to run
            driveLeftBack.setPower(leadingPowerLeft);
            driveLeftFront.setPower(trailingPowerLeft);
        } else {
            driveLeftFront.setPower(0);
            driveLeftBack.setPower(0);
        }
        if (leadingPowerRight > 0.1) {
            driveRightBack.setPower(trailingPowerRight);
            driveRightFront.setPower(leadingPowerRight);
        } else if (leadingPowerRight < -0.1) {
            driveRightBack.setPower(leadingPowerRight);
            driveRightFront.setPower(trailingPowerRight);
        } else {
            driveRightBack.setPower(0);
            driveRightFront.setPower(0);
        }

        //set LED states
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


        //takes input from buttons for spin motors
        if (gamepad2.a) {                   //collect debris
            spin.setPower(-1);

            if (blueTeam) {
                boxPower = boxMovingUp;
            } else {
                boxPower = boxMovingDown;
            }

        } else if (gamepad2.y) {            //eject or sweep away debris

            spin.setPower(1);
            boxPower = boxStopMoving;

        } else if (gamepad1.x) {

            boxPower = boxMovingUp;
            spin.setPower(0);

        } else if (gamepad1.b) {

            boxPower = boxMovingDown;
            spin.setPower(0);

        } else {

            spin.setPower(0);
            boxPower = boxStopMoving;
        }

        box.setPosition(boxPower);

        //window wiper code
        if (gamepad1.a) {
            windowWiperPosition = windowWiperOpened;
        } else {
            windowWiperPosition = windowWiperClosed;
        }

        windowWiper.setPosition(windowWiperPosition);

        //takes input from bumpers and triggers for the locking grab motors set individually
        if (gamepad1.right_bumper) {
            grabRight.setPosition(grabRightUp);
        } else if (gamepad1.right_trigger > .3) {       //these triggers are considered axis,
            grabRight.setPosition(grabRightDown);       //but we effectively utilize them as
        }                                               //buttons by using them like this


        if (gamepad1.left_bumper) {
            grabLeft.setPosition(grabLeftUp);
        } else if (gamepad1.left_trigger > .3) {
            grabLeft.setPosition(grabLeftDown);
        }

        //hang servo extention
        if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3 && this.getRuntime() > 90)
        {
            hangPosition = hangOut;
        }
        else if (gamepad2.left_bumper && gamepad2.right_bumper){
            hangPosition = hangIn;
        }
        hangArms.setPosition(hangPosition);

        //all clear servo extension
        if (gamepad2.dpad_right){
            allClearPosition = allClearUp
        } else if (gamepad2.dpad_left) {
          allClearPosition = allClearDown
        }
        allClear.setPosition(allClearPosition);

        // Arm and Winch extension

        if ((gamepad2.dpad_up || gamepad1.dpad_up) && armsNotExtended) {      //moves arms and winches with d-pad buttons,

            arms.setPower(-1);

        } else if ((gamepad2.dpad_down || gamepad1.dpad_down) && armsNotRetracted) {

            arms.setPower(1);
        } else {
            arms.setPower(0);
        }

        if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {    //allow manual
            leftWinchSpeed = gamepad2.left_stick_y;                         //override of the winch
        }                                                                   //motors when driver
        if (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1) {  //wants control
            rightWinchSpeed = gamepad2.right_stick_y;
        } else {
            leftWinchSpeed = 0;
            rightWinchSpeed = 0;
        }

        rightWinch.setPower(rightWinchSpeed);           //sets power of the winches to the
        leftWinch.setPower(leftWinchSpeed);             //specified power

    }//finish loop

}//finish program
