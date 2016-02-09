package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.List;
import java.util.ArrayList;
/**
 * Created by Roarbots on 10/18/2015.
 */
public abstract class TeleOpResQCommon extends TeleOpCommonV2 {
    // Servo testServo;

    // double servoPosition = 0;
    String motor1 = "motor1";
    String motor2 = "motor2";
    String motor3 = "motor3";
    String motor4 = "motor4";
    String motor5 = "motor5";

    public List<DcMotor> leftMotors;
    public List<DcMotor> rightMotors;
    public DcMotor treadAngleMotor;

    private final double wheelSize = 6.0;
    private final double driveTrainRatio = 40.0 / 1.0;


    @Override
    public void init(){
        leftMotors = new ArrayList<DcMotor>();
        rightMotors = new ArrayList<DcMotor>();

        super.init();
        hardwareMap.dcMotor.get(motor5);

        leftMotors.add(hardwareMap.dcMotor.get(motor1));
        leftMotors.add(hardwareMap.dcMotor.get(motor3));
        rightMotors.add(hardwareMap.dcMotor.get(motor2));
        rightMotors.add(hardwareMap.dcMotor.get(motor4));
        treadAngleMotor = hardwareMap.dcMotor.get(motor5);

        setupRobotParameters(wheelSize, driveTrainRatio);

        rightMotors.get(0).setDirection(DcMotor.Direction.FORWARD);
        rightMotors.get(0).setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotors.get(1).setDirection(DcMotor.Direction.FORWARD);
        rightMotors.get(1).setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftMotors.get(0).setDirection(DcMotor.Direction.REVERSE);
        leftMotors.get(0).setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftMotors.get(1).setDirection(DcMotor.Direction.REVERSE);
        leftMotors.get(1).setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        treadAngleMotor.setDirection(DcMotor.Direction.REVERSE);
        treadAngleMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //  // Setup our servos here
        // testServo = hardwareMap.servo.get(servo1);
    }

    public void setLeftPower(double leftMotorPower) {

        super.setLeftPower(leftMotorPower);
    }

    public void setRightPower(double rightMotorPower) {
        super.setRightPower(rightMotorPower);
    }
}
