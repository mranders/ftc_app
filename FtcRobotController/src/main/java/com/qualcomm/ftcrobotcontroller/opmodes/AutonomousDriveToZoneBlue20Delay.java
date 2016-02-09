package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Roarbots on 10/18/2015.
 */
public class AutonomousDriveToZoneBlue20Delay extends AutonomousResQCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        RightMoustache.setPosition(1.0);
        LeftMoustache.setPosition(0.0);
        theOneThatFlips.setPosition(0.0);

        setupRobotParameters(6.0, 1.0, 40.0);

        reset();

        waitForStart();

        sleep(20000);

        drive(96,0.4);
    }
}