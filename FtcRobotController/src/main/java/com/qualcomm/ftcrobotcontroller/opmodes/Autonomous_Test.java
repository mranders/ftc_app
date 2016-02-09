package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Roarbots on 1/29/2016.
 */
public class Autonomous_Test extends AutonomousResQCommon {
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

        //drive(1, 0.5);
        turnInPlace(90.0, 0.5);
        debugLogMotors();

        turnInPlace(-90.0, 0.4);
        debugLogMotors();

        turnInPlace(45.0, 0.3);
        debugLogMotors();

        turnInPlace(-30.0, 0.2);
        debugLogMotors();

        turnInPlace(-15.0, 0.1);
        debugLogMotors();

        sleep(3000);

        turnInPlace(120.0, 0.7);
        debugLogMotors();

        while (elapsedTime.time() < 30.0){
            telemetry.addData("Heading", gyroSensor.getHeading());
//            telemetry.addData("Integrated", gyroSensor.getIntegratedZValue());
            DbgLog.msg("Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);
            Thread.sleep(100);
        }

    }
}
