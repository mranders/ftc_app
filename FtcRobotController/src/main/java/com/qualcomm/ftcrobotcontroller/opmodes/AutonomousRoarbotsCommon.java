package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Roarbots on 11/20/2015.
 */
public abstract class AutonomousRoarbotsCommon extends LinearOpMode {

    protected List<DcMotor> leftMotors = new ArrayList<DcMotor>();
    protected List<DcMotor> rightMotors = new ArrayList<DcMotor>();

    protected List<DcMotor> allMotors = new ArrayList<DcMotor>();

    // Limits constants
    final static double SERVO_MIN_RANGE  = 0.20;
    final static double SERVO_MAX_RANGE  = 0.90;
    final static double MOTOR_MIN_RANGE  = -1.0;
    final static double MOTOR_MAX_RANGE  = 1.0;

    @Override
    public void runOpMode() throws InterruptedException{
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
    
    public void setLeftPosition(int primaryCount) {
        int start0 = leftMotors.get(0).getCurrentPosition();
        int start1 = leftMotors.get(1).getCurrentPosition();
        int target0 = start0 + primaryCount;
        int target1 = start1 + (int)(primaryCount * (1090.0/1124.0));
        DbgLog.msg("Setting Lmotor0 to "+target0);
        leftMotors.get(0).setTargetPosition(target0);
        DbgLog.msg("Setting Lmotor1 to " + target1);
        leftMotors.get(1).setTargetPosition(target1);
    }

    public void setRightPosition(int primaryCount) {
        int start0 = rightMotors.get(0).getCurrentPosition();
        int start1 = rightMotors.get(1).getCurrentPosition();
        int target0 = start0 + primaryCount;
        int target1 = start1 + (int)(primaryCount * (1090.0/1124.0));
        DbgLog.msg("Setting Rmotor0 to "+target0);
        rightMotors.get(0).setTargetPosition(target0);
        DbgLog.msg("Setting Rmotor1 to " + target1);
        rightMotors.get(1).setTargetPosition(target1);
    }
    
}
