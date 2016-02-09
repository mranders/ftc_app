package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Roarbots on 1/29/2016.
 */
public class Autonomous_TestRunToPos extends AutonomousResQCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime elapsedTime = new ElapsedTime();

        super.runOpMode();
        RightMoustache.setPosition(1.0);
        LeftMoustache.setPosition(0.0);
        theOneThatFlips.setPosition(0.0);

        setupRobotParameters(6.0, 1.0, 40.0);

        reset();
        elapsedTime.reset();
        int sleepCount = 0;
        telemetry.addData("Gyro:", "Calibrating");
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(50);
            sleepCount++;
        }
        telemetry.addData("GyroCalibTime", elapsedTime.time());
        DbgLog.msg("Finished calibration in " + elapsedTime + " seconds, slept " + sleepCount + " times");
        waitForStart();
        elapsedTime.reset();


        debugLogMotors();
        DbgLog.msg("  0 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        //drive(1, 0.5);
        turnInPlaceEncoder(90.0, 0.5);
        debugLogMotors();
        DbgLog.msg(" 90 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        turnInPlaceEncoder(-90.0, 0.4);
        debugLogMotors();
        DbgLog.msg("  0 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        turnInPlaceEncoder(45.0, 0.3);
        debugLogMotors();
        DbgLog.msg(" 45 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        turnInPlaceEncoder(-30.0, 0.2);
        debugLogMotors();
        DbgLog.msg(" 15 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        turnInPlaceEncoder(-15.0, 0.1);
        debugLogMotors();
        DbgLog.msg("  0 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        sleep(3000);
        DbgLog.msg("  0 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        turnInPlaceEncoder(120.0, 0.7);
        debugLogMotors();

        DbgLog.msg("120 Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);

        while (elapsedTime.time() < 30.0){
            telemetry.addData("Heading", gyroSensor.getHeading());
//            telemetry.addData("Integrated", gyroSensor.getIntegratedZValue());
            DbgLog.msg("Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);
            Thread.sleep(100);
        }

    }
}
