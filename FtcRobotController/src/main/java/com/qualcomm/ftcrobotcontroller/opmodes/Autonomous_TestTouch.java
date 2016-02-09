package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Roarbots on 1/29/2016.
 */
public class Autonomous_TestTouch extends AutonomousResQCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime elapsedTime = new ElapsedTime();

        super.runOpMode();
        RightMoustache.setPosition(1.0);
        LeftMoustache.setPosition(0.0);
        theOneThatFlips.setPosition(0.0);

        setupRobotParameters(6.0, 1.0, 40.0);

        reset();
        waitForStart();
        elapsedTime.reset();

        while (elapsedTime.time() < 30.0){
            telemetry.addData("TouchV", armAngleLimitSensor.getValue());
            telemetry.addData("TouchP", armAngleLimitSensor.isPressed());
            telemetry.addData("TouchS", armAngleLimitSensor.toString());
            Thread.sleep(100);
        }

    }
}
