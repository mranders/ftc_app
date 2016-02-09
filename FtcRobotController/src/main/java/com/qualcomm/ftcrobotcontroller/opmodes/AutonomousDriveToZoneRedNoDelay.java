package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.hardware.ModernRoboticsI2cGyro;

/**
 * Created by Roarbots on 10/18/2015.
 */
public class AutonomousDriveToZoneRedNoDelay extends AutonomousResQCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        RightMoustache.setPosition(1.0);
        LeftMoustache.setPosition(0.0);
        theOneThatFlips.setPosition(0.0);

        setupRobotParameters(6.0, 1.0, 40.0);

        reset();

        waitForStart();

        drive(12,0.5);
    }
}