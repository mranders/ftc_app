package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Roarbots on 9/20/2015.
 */
public class PublicDemo extends OpMode {
    private DcMotor motorRight;
    private DcMotor motorRight2;
    private DcMotor motorLeft;
    private DcMotor motorLeft2;
    private static final double THROTTLE_FACTOR = 0.2;

    private final static String motor1 = "motor1";
    private final static String motor2 = "motor2";
    private final static String motor3 = "motor3";
    private final static String motor4 = "motor4";

    // private Servo testServo;
    private double servoPosition = 0;
    private final static String servo1 = "servo1";
    private final static double SERVO_MIN_RANGE  = 0.20;
    private final static double SERVO_MAX_RANGE  = 0.90;
    private final static double diffServoPos = 0.1;

    // Limits constants
    private final static double MOTOR_MIN_RANGE  = -1.0;
    private final static double MOTOR_MAX_RANGE  = 1.0;
    private boolean pushOnceA = false;
    private boolean pushOnceY = false;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        motorRight = hardwareMap.dcMotor.get(motor2);
        motorRight2 = hardwareMap.dcMotor.get(motor4);
        motorLeft = hardwareMap.dcMotor.get(motor1);
        motorLeft2 = hardwareMap.dcMotor.get(motor3);

        // Initializing the controllers
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);

        //  // Setup our servos here
        // testServo = hardwareMap.servo.get(servo1);

        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

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
         // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
         // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        double left_stick = gamepad1.left_stick_y;
        double right_stick = gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        left_stick = Range.clip(left_stick, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        right_stick = Range.clip(right_stick, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        left_stick = THROTTLE_FACTOR * left_stick;
        right_stick = THROTTLE_FACTOR * right_stick;
        // servoPosition = Range.clip(servoPosition, SERVO_MIN_RANGE, SERVO_MAX_RANGE);


        // write the values to the motors
        if(Math.abs(left_stick) > 0.05) {
            motorLeft.setPower(left_stick);
            motorLeft2.setPower(left_stick);
        } else {
            motorLeft.setPower(0.0);
            motorLeft2.setPower(0.0);
        }
        if(Math.abs(right_stick) > 0.05) {
            motorRight.setPower(right_stick);
            motorRight2.setPower(right_stick);
        } else {
            motorRight.setPower(0.0);
            motorRight2.setPower(0.0);
        }

//        if (gamepad2.a) {
//            if (!pushOnceRB) {
//                // if the A button is pushed on gamepad2, increment the position of
//                // the arm servo
//                servoPosition += diffServoPos;
//                pushOnceRB = true;
//            }
//        }
//        else {
//            pushOnceRB = false;
//        }
//
//        if (gamepad2.y) {
//            if (!pushOnceY) {
//                // if the Y button is pushed on gamepad2, increment the position of
//                // the arm servo
//                servoPosition -= diffServoPos;
//                pushOnceY = true;
//            }
//        }
//        else {
//            pushOnceY = false;
//        }
        // testServo.setPosition(servoPosition);


        // Update the reads after some loops, when the command has successfully propagated through.
        telemetry.addData("Text", "free flow text");
        telemetry.addData("left motor", motorLeft.getPower());
        telemetry.addData("right motor", motorRight.getPower());
        telemetry.addData("RunMode: ", motorLeft.getChannelMode().toString());
    }
}