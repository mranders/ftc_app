package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;

/**
 * Created by Roarbots on 11/20/2015.
 */
public class AutonomousCommon extends AutonomousRoarbotsCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

    }

    private double myWheelDiameter;
    private double myDriveTrainRatio;
    private double myMotorRatio;
    private static final double encoderClicksPerRev = 28;
    private static double clicksPerInch;

    /*
     * Sets up the parameters of the robot to use in our functions
     *
     * wheelSize = Diameter of the wheel in inches
     */
    public void setupRobotParameters(double newWheelDiameter, double newDriveTrainRatio, double newMotorRatio) {
        myWheelDiameter = newWheelDiameter;
        myDriveTrainRatio = newDriveTrainRatio;
        myMotorRatio = newMotorRatio;
        clicksPerInch = (encoderClicksPerRev * myMotorRatio * myDriveTrainRatio) / (Math.PI * myWheelDiameter);
    }


    public void drive(int inches, double speed) throws InterruptedException {
        int firstPosition;
        int slowStart;
        int slowStop;
        int fullSpeed;
        setLeftPower(0);
        setRightPower(0);


        int clicksForDistance = (int) (inches * clicksPerInch);
        int position = leftMotors.get(0).getCurrentPosition();
        fullSpeed = position + clicksForDistance;
//        if(clicksForDistance >= 1000)
//        {
            slowStart = position + 500;
//            fullSpeed = slowStart + clicksForDistance - 1000;
            slowStop = fullSpeed + 500;
//        }
//        else
//        {
//            slowStart = position + clicksForDistance / 2;
//            fullSpeed = slowStart;
//            slowStop = fullSpeed + clicksForDistance / 2;
//        }
        firstPosition = position;
        DbgLog.msg("drive: firstPos:"+firstPosition+" targPos:"+fullSpeed+" speed:"+speed);

//        setLeftPower(speed / 2.0);
//        setRightPower(speed / 2.0);
//
//        while (position < slowStart) {
//            outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, speed);
//            position = leftMotors.get(0).getCurrentPosition();
//        }
//
        setLeftPower(speed);
        setRightPower(speed);

        // To handle negative travel, we expect negative distance and negative speed, and this condition handles both
        while ((firstPosition < fullSpeed) ? position < fullSpeed : position > fullSpeed) {
            setLeftPower(speed);
            setRightPower(speed);
            waitForNextHardwareCycle();
            debugLogAllDriveMotorPosition();
            outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, speed);
            position = leftMotors.get(0).getCurrentPosition();
        }

//        setLeftPower(speed / 2.0);
//        setRightPower(speed / 2.0);
//
//        while (position < slowStop) {
//            outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, speed);
//            position = leftMotors.get(0).getCurrentPosition();
//        }
        debugLogAllDriveMotorPosition();
        outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, speed);

        setRightPower(0);
        setLeftPower(0);
        sleep(10);
        setRightPower(0);
        setLeftPower(0);
        outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, speed);

    }

    public void LBasedTurn(double speed, int degrees) throws InterruptedException {
        // if speed is positive, Left wheels will turn forward
        // if speed is negative, Left wheels will turn backward
        double inchesPerDegree = 56.636 / 360;
        double inchesToMove = degrees * inchesPerDegree;
        int clicksForDistance =  (int) (inchesToMove * clicksPerInch);
        int position = leftMotors.get(0).getCurrentPosition();

        double finalPosition = position + clicksForDistance;

        setLeftPower(speed);
        setRightPower(-speed);

        while (position < finalPosition) {
            telemetry.addData("current postion: ", position);
            telemetry.addData("final Position: ", finalPosition);
            telemetry.addData("Degrees: ", degrees);
            telemetry.addData("left speed: ", speed);
            telemetry.addData("right speed: ", -speed);
            position = leftMotors.get(0).getCurrentPosition();
        }

        setRightPower(0);
        setLeftPower(0);
        sleep(10);
        setRightPower(0);
        setLeftPower(0);
    }

    public void drive2(int inches, double speed, double turnRatio, boolean turnRight) throws InterruptedException {
        int firstPosition;
        int slowStart=0;
        int slowStop=0;
        int fullSpeed=0;
        setRightPower(0);
        setLeftPower(0);
        double slowSpeed=speed * 0.7;

        int clicksForDistance = (int) (inches * clicksPerInch);
        int position = leftMotors.get(0).getCurrentPosition();
        if(clicksForDistance >= 1000)
        {
            slowStart = position + 500;
            fullSpeed = slowStart + clicksForDistance - 1000;
            slowStop = fullSpeed + 500;
        }
        else
        {
            slowStart = position + clicksForDistance / 2;
            fullSpeed = slowStart;
            slowStop = fullSpeed + clicksForDistance / 2;
        }
        firstPosition = position;

        setRightPower(speed / 2.0);
        setLeftPower(slowSpeed / 2.0);

        while (position < slowStart) {
            outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, slowSpeed);
            waitOneFullHardwareCycle();
            position = leftMotors.get(0).getCurrentPosition();
        }

        setRightPower(speed);
        setLeftPower(slowSpeed);

        while (position < fullSpeed) {
            outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, slowSpeed, speed);
            waitOneFullHardwareCycle();
            position = leftMotors.get(0).getCurrentPosition();
        }

        setRightPower(speed / 2.0);
        setLeftPower(slowSpeed / 2.0);

        while (position < slowStop) {
            outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, slowSpeed, speed);
            waitOneFullHardwareCycle();
            position = leftMotors.get(0).getCurrentPosition();
        }
        outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, slowSpeed, speed);

        setRightPower(0);
        setLeftPower(0);
        sleep(10);
        setRightPower(0);
        setLeftPower(0);
        outputDriveData(firstPosition, slowStart, fullSpeed, slowStop, position, speed, slowSpeed);

    }

    public void debugLogMotors(){
        for (DcMotor motor : allMotors){
            DbgLog.msg(motor.getConnectionInfo() + " power:" + motor.getPower() + " direction:" + motor.getDirection() + " mode:" + motor.getMode()
                        + " currPos:"+ motor.getCurrentPosition() + " targPos:" + motor.getTargetPosition() + " isBusy:" + motor.isBusy());
        }
    }


    public void debugLogAllDriveMotorPosition(){
        DbgLog.msg("cur/trg/pwr/bsy L1:" + motorSummaryString( leftMotors.get(0)) + " L2:" + motorSummaryString( leftMotors.get(1))
                                + " R1:" + motorSummaryString(rightMotors.get(0)) + " R2:" + motorSummaryString(rightMotors.get(1)));
    }

    public void debugLogMotorPosition(String prefix, DcMotor motor){
        DbgLog.msg(prefix + motorSummaryString(motor));
    }
    public String motorSummaryString(DcMotor motor) {
        RunMode mode = motor.getMode();
        String modeAbbr;
        if (mode == RunMode.RUN_TO_POSITION)
            modeAbbr = "R_T_POS";
        else if (mode == RunMode.RUN_WITHOUT_ENCODERS)
            modeAbbr = "R_No_ENC";
        else if (mode == RunMode.RUN_USING_ENCODERS)
            modeAbbr = "R_UseENC";
        else if (mode == RunMode.RESET_ENCODERS)
            modeAbbr = "RES_ENC";
        else
            modeAbbr = "JUNK";
        String retval = motor.getCurrentPosition() + "/" + motor.getTargetPosition() + "/" + motor.getPower() + "/" + modeAbbr + "/" + (motor.isBusy() ? "busy" : "");
        return retval;
    }
    public void outputDriveData(int position, int slowStart, int fullSpeed, int slowStop, int currentPosition, double speed, double rightSpeed) {
        telemetry.clearData();
        telemetry.addData("Start Position", position);
        telemetry.addData("Slow Start Position", slowStart);
        telemetry.addData("Full Speed Position", fullSpeed);
        telemetry.addData("Stop Position", slowStop);
        telemetry.addData("Current", currentPosition);
        telemetry.addData("LeftSpeed", speed);
        telemetry.addData("RightSpeed", rightSpeed);
    }
}