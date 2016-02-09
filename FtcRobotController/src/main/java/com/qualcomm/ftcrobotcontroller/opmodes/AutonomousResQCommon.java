package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Roarbots on 11/20/2015.
 */
public class AutonomousResQCommon extends AutonomousCommon {

    private static final int MIN_LOOP_COUNT = 10;
    private List<DcMotor> encoderMotors = new ArrayList<DcMotor>();
    // Servo testServo;
    public static final int ENCODER_ALL_ALL     = 0x00333;
    public static final int ENCODER_RIGHT_ALL   = 0x00003;
    public static final int ENCODER_RIGHT_FRONT = 0x00001;
    public static final int ENCODER_RIGHT_REAR  = 0x00002;
    public static final int ENCODER_LEFT_ALL    = 0x00030;
    public static final int ENCODER_LEFT_FRONT  = 0x00010;
    public static final int ENCODER_LEFT_REAR   = 0x00020;
    public static final int ENCODER_TAPE_ALL    = 0x00300;
    public static final int ENCODER_TAPE_ANGLE  = 0x00100;
    public static final int ENCODER_TAPE_DIST   = 0x00200;

    public static final double SERVO_RIGHT_MUSTACHE_INIT_POSITION = 0.0;
    public static final double SERVO_LEFT_MUSTACHE_INIT_POSITION = 0.0;
    public static final double SERVO_CLIMBER_ARM_INIT_POSITION = 0.0;

    // double servoPosition = 0;
    String motor1 = "motor1";//left
    String motor2 = "motor2";//left
    String motor3 = "motor3";//right
    String motor4 = "motor4";//right
    String motor5 = "motor5";//extender lifter upper and lower downer
    String motor6 = "motor6";//extender

    String servo1 = "servo1";//The one that flips
    String servo2 = "servo2";// L Moustache
    String servo3 = "servo3";// R Moustache

    String gyro = "gyro1";
    String touch = "touch1";
    public DcMotor tapeAngleMotor;
    public DcMotor tapePowerMotor;

    public Servo theOneThatFlips;//Climber placer
    public Servo LeftMoustache; //left Moustache
    public Servo RightMoustache; //right Moustache

    public GyroSensor gyroSensor;
    public TouchSensor armAngleLimitSensor;

    private final double wheelSize = 6.0;
    private final double driveTrainRatio = 40.0 / 1.0;
    protected final static double TIMEOUT_FOR_TURN_EXECUTION = 4.0;


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotors = new ArrayList<DcMotor>();
        rightMotors = new ArrayList<DcMotor>();
        super.runOpMode();
        rightMotors.add(hardwareMap.dcMotor.get(motor1));
        rightMotors.add(hardwareMap.dcMotor.get(motor2));
        leftMotors.add(hardwareMap.dcMotor.get(motor3));
        leftMotors.add(hardwareMap.dcMotor.get(motor4));
        tapeAngleMotor = hardwareMap.dcMotor.get(motor5);
        tapePowerMotor = hardwareMap.dcMotor.get(motor6);

        allMotors.addAll(rightMotors);
        allMotors.addAll(leftMotors);
        allMotors.add(tapeAngleMotor);
        allMotors.add(tapePowerMotor);

        theOneThatFlips = hardwareMap.servo.get(servo1);
        LeftMoustache = hardwareMap.servo.get(servo2);
        RightMoustache = hardwareMap.servo.get(servo3);

        gyroSensor = hardwareMap.gyroSensor.get(gyro);
        armAngleLimitSensor = hardwareMap.touchSensor.get(touch);

        for (DcMotor singleMotor : rightMotors) {
            singleMotor.setDirection(DcMotor.Direction.FORWARD);
            waitOneFullHardwareCycle();
            singleMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            waitOneFullHardwareCycle();
        }
        for (DcMotor singleMotor : leftMotors) {
            singleMotor.setDirection(DcMotor.Direction.REVERSE);
            waitOneFullHardwareCycle();
            singleMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            waitOneFullHardwareCycle();
        }

        tapeAngleMotor.setDirection(DcMotor.Direction.REVERSE);
        waitOneFullHardwareCycle();
        tapeAngleMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        tapePowerMotor.setDirection(DcMotor.Direction.FORWARD);
        waitOneFullHardwareCycle();
        tapePowerMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();

        setLeftPower(0.0);
        setRightPower(0.0);
        setTapeAnglePower(0.0);
        setTapeMotorPower(0.0);
        waitOneFullHardwareCycle();
    }

    public void setLeftPower(double leftMotorPower) {super.setLeftPower(leftMotorPower);}

    public void setRightPower(double rightMotorPower) {super.setRightPower(rightMotorPower);}

    public void setTapeAnglePower(double power){tapeAngleMotor.setPower(power);}
    public void setTapeMotorPower(double power) {tapePowerMotor.setPower((power));}

    public void reset () throws InterruptedException {
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            DbgLog.msg("gyro isCalibrating");
            sleep(50);
        }
        for (DcMotor singleMotor : rightMotors) {
            singleMotor.setDirection(DcMotor.Direction.FORWARD);
            waitOneFullHardwareCycle();
        }
        for (DcMotor singleMotor : leftMotors) {
            singleMotor.setDirection(DcMotor.Direction.REVERSE);
            waitOneFullHardwareCycle();
        }
        tapeAngleMotor.setDirection(DcMotor.Direction.REVERSE);
        waitOneFullHardwareCycle();
        tapePowerMotor.setDirection(DcMotor.Direction.FORWARD);
        waitOneFullHardwareCycle();
        tapePowerMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();

        // Rest encoders and set to RUN_USING_ENCODERS, and wait for encoders to reset
        resetEncodersAndWaitClear(ENCODER_ALL_ALL);
        modeChangeMotors(ENCODER_LEFT_ALL | ENCODER_RIGHT_ALL, DcMotorController.RunMode.RUN_USING_ENCODERS);
        modeChangeMotorsAndWait(ENCODER_TAPE_ANGLE | ENCODER_TAPE_DIST, DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void modeChangeMotors(int motorMask, DcMotorController.RunMode mode) throws InterruptedException {
        encoderMotors.clear();
        if ((motorMask & ENCODER_LEFT_FRONT)  != 0) {encoderMotors.add(leftMotors.get(0));  DbgLog.msg("encoderList.add " + leftMotors.get(0) );}
        if ((motorMask & ENCODER_LEFT_REAR)   != 0) {encoderMotors.add(leftMotors.get(1));  DbgLog.msg("encoderList.add " + leftMotors.get(1) );}
        if ((motorMask & ENCODER_RIGHT_FRONT) != 0) {encoderMotors.add(rightMotors.get(0)); DbgLog.msg("encoderList.add " + rightMotors.get(0));}
        if ((motorMask & ENCODER_RIGHT_REAR)  != 0) {encoderMotors.add(rightMotors.get(1)); DbgLog.msg("encoderList.add " + rightMotors.get(1));}
        if ((motorMask & ENCODER_TAPE_ANGLE)  != 0) {encoderMotors.add(tapeAngleMotor);     DbgLog.msg("encoderList.add " + tapeAngleMotor    );}
        if ((motorMask & ENCODER_TAPE_DIST)   != 0) {encoderMotors.add(tapePowerMotor);     DbgLog.msg("encoderList.add " + tapePowerMotor    );}

        for (DcMotor motor : encoderMotors) {
            motor.setMode(mode);
        }
        DbgLog.msg("Motors set to mode: " + mode);
    }

    public void modeChangeMotorsAndWait(int motorMask, DcMotorController.RunMode mode) throws InterruptedException {
        modeChangeMotors(motorMask, mode);

        waitOneFullHardwareCycle();
        DbgLog.msg("modeChangeMotorsAndWait done");
    }

    public void resetEncodersAndWaitClear(int motorMask) throws InterruptedException {
        modeChangeMotorsAndWait(motorMask, DcMotorController.RunMode.RESET_ENCODERS);

        boolean allZero = false;
        while (!allZero) {
            allZero = true;
            waitForNextHardwareCycle();
            for (DcMotor motor : encoderMotors) {
                if (motor.getCurrentPosition() != 0)
                    allZero = false;
            }
        }
        DbgLog.msg("resetEncodersAndWaitClear done");
    }

    public void turnInPlace(double degrees, double speed) throws InterruptedException {
        double rightSpeed;
        double leftSpeed;

        setLeftPower(0);
        setRightPower(0);

        double driveSteering;
        double midPower = 0.0;
        double driveGain = 0.01;

        int startHeading = gyroSensor.getHeading();
        int currentHeading = startHeading;
        if (startHeading > 180) startHeading -= 360;

        int headingError;
        int targetHeading = (int) (startHeading + degrees);
        if (targetHeading > 180) targetHeading -= 360;

        DbgLog.msg("Heading:" + currentHeading + " target:" + targetHeading + " start:" + startHeading + " degrees:" + degrees);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        ElapsedTime elapsedSettling = new ElapsedTime();
        elapsedSettling.reset();

        double settlingTime = 0.5;
        boolean settling = false;
        boolean nextSettling = false;
        do {
            waitOneFullHardwareCycle();
            currentHeading = gyroSensor.getHeading();
            if (currentHeading > 180) {
                currentHeading = currentHeading - 360;
            }
            headingError = targetHeading - currentHeading;

            driveSteering = headingError * driveGain;
            leftSpeed = midPower - driveSteering;
            if (leftSpeed < -speed) leftSpeed = -speed;
            if (leftSpeed >  speed) leftSpeed =  speed;

            rightSpeed = midPower + driveSteering;
            if (rightSpeed < -speed) rightSpeed = -speed;
            if (rightSpeed >  speed) rightSpeed =  speed;

            setLeftPower(leftSpeed);
            setRightPower(rightSpeed);

            nextSettling = (Math.abs(headingError) < 5);
            if (!settling & nextSettling) {
                elapsedSettling.reset();
            }
            settling = nextSettling;
            DbgLog.msg("Heading:" + currentHeading + " target:" + targetHeading + " start:" + startHeading + " degrees:" + degrees + " headingError:" + headingError + " driveSteering:" + driveSteering + " Right:" + rightSpeed + " Left:" + leftSpeed + " elapsed:" + elapsedTime);
            //debugLogMotors();
        } while (
                ((Math.abs(headingError) > 1) || (!settling) || (elapsedSettling.time() < settlingTime))
                        &&
                (elapsedTime.time() < TIMEOUT_FOR_TURN_EXECUTION)
                );

        setRightPower(0);
        setLeftPower(0);
        waitOneFullHardwareCycle();
        setRightPower(0);
        setLeftPower(0);

    }

    public void turnInPlaceEncoder(double degrees, double speed) throws InterruptedException {
        double scaleConstant = 1124.0/90.0;

        setLeftPower(0);
        setRightPower(0);
        waitOneFullHardwareCycle();

        modeChangeMotorsAndWait(ENCODER_LEFT_ALL | ENCODER_RIGHT_ALL, DcMotorController.RunMode.RUN_TO_POSITION);

        int startHeading = gyroSensor.getHeading();
        int currentHeading = startHeading;
        if (startHeading > 180) startHeading -= 360;

        int headingError;
        int targetHeading = (int) (startHeading + degrees);
        if (targetHeading > 180) targetHeading -= 360;

        setLeftPosition ((int) (-degrees * scaleConstant));
        setRightPosition((int) (degrees * scaleConstant));

        int leftTarget = leftMotors.get(0).getTargetPosition();
        int rightTarget = rightMotors.get(0).getTargetPosition();

        DbgLog.msg("Heading:" + currentHeading + " target:" + targetHeading + " start:" + startHeading + " degrees:" + degrees);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        boolean motorsBusy = false;
        boolean busyStarted = false;
        ElapsedTime elapsedSettling = new ElapsedTime();
        elapsedSettling.reset();

        double settlingTime = 0.15;
        boolean settling = false;

        do {
            waitOneFullHardwareCycle();
            currentHeading = gyroSensor.getHeading();
            if (currentHeading > 180) {
                currentHeading = currentHeading - 360;
            }
            headingError = targetHeading - currentHeading;

            setLeftPower(speed);
            setRightPower(speed);

            DbgLog.msg("Heading:" + currentHeading + " target:" + targetHeading + " start:" + startHeading + " degrees:" + degrees
                    + " headingError:" + headingError
                    + " Right:" + rightMotors.get(0).getCurrentPosition() + " Left:" + leftMotors.get(0).getCurrentPosition() + " elapsed:" + elapsedTime);
            debugLogAllDriveMotorPosition();

            for (DcMotor motor : allMotors) {
                motorsBusy = motorsBusy || motor.isBusy();
            }
            if (busyStarted && !motorsBusy) {
                settling = true;
                elapsedSettling.reset();
            }
            busyStarted |= motorsBusy;
        } while ((!busyStarted || motorsBusy || (settling && (elapsedSettling.time() < settlingTime))) && (elapsedTime.time() < TIMEOUT_FOR_TURN_EXECUTION));

        setRightPower(0);
        setLeftPower(0);
        waitOneFullHardwareCycle();
        setRightPower(0);
        setLeftPower(0);

    }

    public void raiseArmToStop(double speed) throws InterruptedException {
        telemetry.clearData();
        telemetry.addData("Method1","raiseArmToStop("+speed+")");
        DbgLog.msg("raiseArmToStop begins");
        ElapsedTime watchdogTimer = new ElapsedTime();
        watchdogTimer.reset();

        if (!armAngleLimitSensor.isPressed()) {
            telemetry.addData("Method2", "!limitSensor.isPressed");
            DbgLog.msg("raiseArmToStop begins !armAngleLimitSensor.isPressed()");
            modeChangeMotorsAndWait(ENCODER_TAPE_ANGLE, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            setTapeAnglePower(speed);
            while ((watchdogTimer.time() < 5.0) && !armAngleLimitSensor.isPressed()) {
                setTapeAnglePower(speed);
                waitForNextHardwareCycle();
                debugLogMotorPosition("Raising arm: ", tapeAngleMotor);
            }
            setTapeAnglePower(0.0);
            waitOneFullHardwareCycle();
            DbgLog.msg("tapeAngleCurrPos before reset0:" + tapeAngleMotor.getCurrentPosition());
            modeChangeMotorsAndWait(ENCODER_TAPE_ANGLE, DcMotorController.RunMode.RESET_ENCODERS);
            DbgLog.msg("tapeAngleCurrPos before reset1:" + tapeAngleMotor.getCurrentPosition());

            while ((Math.abs(tapeAngleMotor.getCurrentPosition()) > 10) && (watchdogTimer.time() < 5.0)) {
                telemetry.addData("Method2","Wait CurrPos~=0");
                waitForNextHardwareCycle();
                debugLogMotorPosition("tapeAngleMotor: ",tapeAngleMotor);
            }
            setTapeAnglePower(0.0);
            modeChangeMotorsAndWait(ENCODER_TAPE_ANGLE, DcMotorController.RunMode.RUN_TO_POSITION);
            waitOneFullHardwareCycle();
        }
        DbgLog.msg("raiseArmToStop ends, watchDog:" + watchdogTimer.time());
        telemetry.clearData();
    }

    public void lowerArmDegrees(double degrees, double speed) throws InterruptedException {
        telemetry.clearData();
        telemetry.addData("Method1", "lowerArmDegrees(deg=" + degrees + ", spd=" + speed + ")");

        ElapsedTime watchdogTimer = new ElapsedTime();
        boolean motionStarted = false;
        watchdogTimer.reset();
        DbgLog.msg("watchdog:" + watchdogTimer + " cur/trg/pwr/bsy: " + motorSummaryString(tapeAngleMotor));

        double degreesToArmTicks = 4820.0 / 90.0; // Measured 90 degrees as -458 vertical to -5278 horizontal
        double ticksToRotate = degrees * degreesToArmTicks;
        int currPosition = tapeAngleMotor.getCurrentPosition();
        // Must subtract, the negative direction is the one that is legal
        int targPosition = (int)(currPosition - ticksToRotate);
        targPosition = (targPosition > 0) ? 0 : targPosition;
        targPosition = (targPosition < (int)(-90*degreesToArmTicks)) ? (int)(-90*degreesToArmTicks) : targPosition;
        DbgLog.msg("watchdog:" + watchdogTimer + " currPos: " + currPosition + " targPos: " + targPosition + " degree->ticks: " + degreesToArmTicks);

        tapeAngleMotor.setTargetPosition(targPosition);
        DbgLog.msg("watchdog:" + watchdogTimer + " cur/trg/pwr/bsy: " + motorSummaryString(tapeAngleMotor));
        int min_loop_count = 10;
        int loop_count = 0;
        while ((loop_count++ < MIN_LOOP_COUNT) || tapeAngleMotor.isBusy()) {
            setTapeAnglePower(speed);
            waitForNextHardwareCycle();
            DbgLog.msg("watchdog:"+watchdogTimer+" cur/trg/pwr/bsy: " + motorSummaryString(tapeAngleMotor));
        }
        setTapeAnglePower(0.0);
        waitOneFullHardwareCycle();
        setTapeAnglePower(0.0);
        waitOneFullHardwareCycle();
    }

    public void extendArm(double inches, double speed) throws InterruptedException {
        telemetry.clearData();
        telemetry.addData("Method1", "Extend Arm Inches(inches=" + inches + ", spd=" + speed + ")");


        ElapsedTime watchdogTimer = new ElapsedTime();
        boolean motionStarted = false;
        watchdogTimer.reset();
        telemetry.addData("Method1", "Extend Arm Inches(inches=" + inches + ", spd=" + speed + ")");
        DbgLog.msg("watchdog:" + watchdogTimer + " Tape cur/trg/pwr/bsy: " + motorSummaryString(tapePowerMotor));

        double inchesToArmTicks = -1600/3.0;
        double ticksToRotate = inches * inchesToArmTicks;
        int currPosition = tapePowerMotor.getCurrentPosition();
        int targPosition = (int)(currPosition + ticksToRotate);
        //targPosition = (targPosition > 0) ? 0 : targPosition;
       // targPosition = (targPosition < (int)(-90*degreesToArmTicks)) ? (int)(-90*degreesToArmTicks) : targPosition;
        DbgLog.msg("watchdog:" + watchdogTimer + " currPos: " + currPosition + " targPos: " + targPosition + " inches->ticks: " + inchesToArmTicks);

        tapePowerMotor.setTargetPosition(targPosition);
        DbgLog.msg("watchdog:" + watchdogTimer + " cur/trg/pwr/bsy: " + motorSummaryString(tapePowerMotor));
        int loop_count = 0;
        while ((loop_count++ < MIN_LOOP_COUNT) || tapePowerMotor.isBusy()) {
            setTapeMotorPower(speed);
            waitForNextHardwareCycle();
            DbgLog.msg("watchdog:"+watchdogTimer+" cur/trg/pwr/bsy: " + motorSummaryString(tapePowerMotor));
        }
        setTapeMotorPower(0.0);
        waitOneFullHardwareCycle();
        setTapeMotorPower(0.0);
        waitOneFullHardwareCycle();
    }

}

