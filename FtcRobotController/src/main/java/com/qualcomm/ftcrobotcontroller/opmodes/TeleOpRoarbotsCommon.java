package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Roarbots on 10/18/2015.
 */
public abstract class TeleOpRoarbotsCommon extends OpMode {

    // String servo1 = "servo1";
    final static double diffServoPos = 0.1;
    protected List<DcMotor> leftMotors = new ArrayList<DcMotor>();
    protected List<DcMotor> rightMotors = new ArrayList<DcMotor>();

    // Limits constants
    final static double SERVO_MIN_RANGE  = 0.20;
    final static double SERVO_MAX_RANGE  = 0.90;
    final static double MOTOR_MIN_RANGE  = -1.0;
    final static double MOTOR_MAX_RANGE  = 1.0;
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init(){
    }

    public void setLeftPower(double power) {
        for (DcMotor singleMotor : leftMotors) {
            singleMotor.setPower(power);
        }
    }

    public void setRightPower(double power) {
        for (DcMotor singleMotor : rightMotors) {
            singleMotor.setPower(power);
        }
    }

}
