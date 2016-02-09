package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Roarbots on 1/29/2016.
 */
public class Autonomous_Ethan extends AutonomousResQCommon {
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

        drive(12, 0.4);

        turnInPlace(45, 0.4);

        drive(90, 0.4);

        turnInPlace(45, 0.4);

        drive(8, 0.4);

        theOneThatFlips.setPosition(0);
        sleep(250);
        theOneThatFlips.setPosition(1);

        while (elapsedTime.time() < 30.0){
            telemetry.addData("Heading", gyroSensor.getHeading());
//            telemetry.addData("Integrated", gyroSensor.getIntegratedZValue());
            DbgLog.msg("Heading:" + gyroSensor.getHeading() + /*" IntZ:" + gyroSensor.getIntegratedZValue() +*/ " elapsed: " + elapsedTime);
            Thread.sleep(100);
        }

    }
}
