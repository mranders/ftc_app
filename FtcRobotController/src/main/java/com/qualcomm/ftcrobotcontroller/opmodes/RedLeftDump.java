package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Roarbots on 11/1/2015.
 */
public class RedLeftDump extends AutonomousResQCommon  {

    @Override
    public void runOpMode() throws InterruptedException
    {
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

        drive(64, 0.4);
        turnInPlace(-65, 0.4);
        drive(32, 0.4);
        theOneThatFlips.setPosition(1);
        wait(750);
        theOneThatFlips.setPosition(0);
    }
}
