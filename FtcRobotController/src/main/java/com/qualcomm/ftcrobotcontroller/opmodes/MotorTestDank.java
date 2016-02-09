package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import android.provider.MediaStore;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import java.io.IOException;

/**
 * Created by Roarbots on 9/20/2015.
 */
public class MotorTestDank extends OpMode {

//    DcMotor motorRight;
//    DcMotor motorRight2;
//    DcMotor motorLeft;
//    DcMotor motorLeft2;
//    Servo testServo;

    double servoPosition = 0;

    int numOpLoops = 1;
    String motorController1 = "Motor Controller 1";
    String motor1 = "motor1";
    String motor2 = "motor2";
    String motor3 = "motor3";
    String motor4 = "motor4";
    String servoController2 = "Servo Controller 2";
    String servo1 = "servo1";
    final static double diffServoPos = 0.1;

    // Limits constants
    final static double SERVO_MIN_RANGE  = 0.20;
    final static double SERVO_MAX_RANGE  = 0.90;
    final static double MOTOR_MIN_RANGE  = -1.0;
    final static double MOTOR_MAX_RANGE  = 1.0;
    boolean pushOnceA = false;
    boolean pushOnceY = false;

    MediaPlayer player;

  //  player = MediaPlayer.create(Games.this,R.raw.macdonald);



    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */



//    @Override
//    public void start()
//    {
//        super.start();
//        player.start();
//        telemetry.addData("Started Theme Song", "Yes");
//    }
//
//    @Override
//    public void stop()
//    {
//        super.stop();
//        player.stop();
//        telemetry.addData("Stopped Theme Song", "No");
//    }
//
   @Override
    public void init()
    {
//        try {
//            player.setDataSource("John Cena Theme Song.mp3");
//        } catch (IOException e) {
//            telemetry.addData("Theme Song Problem", "ARG!");
//        }
//        player.setLooping(true);
//        player.setVolume(10,10);

//        motorRight = hardwareMap.dcMotor.get(motor2);
//        motorRight2 = hardwareMap.dcMotor.get(motor4);
//        motorLeft = hardwareMap.dcMotor.get(motor1);
//        motorLeft2 = hardwareMap.dcMotor.get(motor3);

//        motorRight.setDirection(DcMotor.Direction.REVERSE);
//        motorLeft.setDirection(DcMotor.Direction.FORWARD);

        // Setup our servos here
//        testServo = hardwareMap.servo.get(servo1);

//        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }
    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
    /*
     * Gamepad 1
     *
     * Gamepad 1 controls the motors via the left stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     */
        if (gamepad1.dpad_left) {
            // Nxt devices start up in "write" mode by default, so no need to switch modes here.
//            motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//            motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
        if (gamepad1.dpad_right) {
            // Nxt devices start up in "write" mode by default, so no need to switch modes here.
//            motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//            motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        double left_stick = gamepad1.left_stick_y;
        double right_stick = gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        left_stick = Range.clip(left_stick, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        right_stick = Range.clip(right_stick, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        servoPosition = Range.clip(servoPosition, SERVO_MIN_RANGE, SERVO_MAX_RANGE);


        // write the values to the motors
        if(Math.abs(left_stick) > 0.1) {
//            motorLeft.setPower(left_stick);
//            motorLeft2.setPower(left_stick);
        } else {
//            motorLeft.setPower(0.0);
//            motorLeft2.setPower(0.0);
        }
        if(Math.abs(right_stick) > 0.1) {
//            motorRight.setPower(right_stick);
//            motorRight2.setPower(right_stick);
        } else {
//            motorRight.setPower(0.0);
//            motorRight2.setPower(0.0);
        }

        if (gamepad2.a) {
            if (!pushOnceA) {
                // if the A button is pushed on gamepad1, increment the position of
                // the arm servo.
                servoPosition += diffServoPos;
                pushOnceA = true;
            }
        }
        else {
            pushOnceA = false;
        }

        if (gamepad2.y) {
            if (!pushOnceY) {
                // if the Y button is pushed on gamepad1, increment the position of
                // the arm servo.
                servoPosition -= diffServoPos;
                pushOnceY = true;

            }
        }
        else {
            pushOnceY = false;
        }
//        testServo.setPosition(servoPosition);


        // Update the reads after some loops, when the command has successfully propagated through.
        telemetry.addData("Text", "free flow text");
//        telemetry.addData("left motor", motorLeft.getPower());
//        telemetry.addData("right motor", motorRight.getPower());
//        telemetry.addData("RunMode: ", motorLeft.getChannelMode().toString());
    }
}