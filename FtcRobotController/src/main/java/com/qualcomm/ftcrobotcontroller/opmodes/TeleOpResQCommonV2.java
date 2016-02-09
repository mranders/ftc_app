package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
/**
 * Created by Roarbots on 10/18/2015.
 */
public abstract class TeleOpResQCommonV2 extends TeleOpCommonV3 {
    // Servo testServo;

    // double servoPosition = 0;
    String motor1 = "motor1";//left
    String motor3 = "motor2";//left
    String motor2 = "motor3";//right
    String motor4 = "motor4";//right
    String motor5 = "motor5";//extender lifter upper and lower downer
    String motor6 = "motor6";//extender

    String servo1 = "servo1";//The one that flips
    String servo2 = "servo2";// L Moustache
    String servo3 = "servo3";// R Moustache

    String gyro1 = "gyro1";
    String armAngleLimitSensorName = "touch1";

    public DcMotor tapeAngleMotor;
    public DcMotor tapePowerMotor;

    public Servo theOneThatFlips;//Climber placer
    public Servo LeftMoustache; //left Moustache
    public Servo RightMoustache; //right Moustache

    public GyroSensor gyroSensor;
    public TouchSensor armAngleLimitSensor;


    private final double wheelSize = 6.0;
    private final double driveTrainRatio = 40.0 / 1.0;


    @Override
    public void init(){
        leftMotors = new ArrayList<DcMotor>();
        rightMotors = new ArrayList<DcMotor>();
        super.init();
        leftMotors.add(hardwareMap.dcMotor.get(motor2));
        leftMotors.add(hardwareMap.dcMotor.get(motor4));
        rightMotors.add(hardwareMap.dcMotor.get(motor1));
        rightMotors.add(hardwareMap.dcMotor.get(motor3));
        tapeAngleMotor = hardwareMap.dcMotor.get(motor5);
        tapePowerMotor = hardwareMap.dcMotor.get(motor6);

        theOneThatFlips = hardwareMap.servo.get(servo1);
        LeftMoustache = hardwareMap.servo.get(servo2);
        RightMoustache = hardwareMap.servo.get(servo3);

        gyroSensor = hardwareMap.gyroSensor.get(gyro1);
        armAngleLimitSensor = hardwareMap.touchSensor.get(armAngleLimitSensorName);

     //   int position = leftMotors.get(0).getCurrentPosition();

        for (DcMotor singleMotor : rightMotors) {
            singleMotor.setDirection(DcMotor.Direction.FORWARD);
            singleMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        for (DcMotor singleMotor : leftMotors) {
            singleMotor.setDirection(DcMotor.Direction.REVERSE);
            singleMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        tapeAngleMotor.setDirection(DcMotor.Direction.REVERSE);
        tapeAngleMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tapePowerMotor.setDirection(DcMotor.Direction.FORWARD);
        tapePowerMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void setLeftPower(double leftMotorPower){super.setLeftPower(leftMotorPower);}
    public void setRightPower(double rightMotorPower){super.setRightPower(rightMotorPower);}

    public void setTapeAnglePower(double power){tapeAngleMotor.setPower(power);}
    public void setTapeMotorPower(double power){tapePowerMotor.setPower(power);}

    public void reset () {
//        for (int i = 0; i < rightMotors.size(); i++) {
//            rightMotors.get(i).setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//            leftMotors.get(i).setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        }
//        tapeAngleMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        tapePowerMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

}