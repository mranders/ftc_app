package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Roarbots on 1/29/2016.
 */
public class Autonomous_ResetArm extends AutonomousResQCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime elapsedTime = new ElapsedTime();

        super.runOpMode();
        RightMoustache.setPosition(1.0);
        LeftMoustache.setPosition(0.0);
        theOneThatFlips.setPosition(0.0);

        setupRobotParameters(6.0, 1.0, 40.0);

        reset();
        boolean stage1True = false;
        boolean stage2True = false;
        boolean stage3True = false;
        boolean stage4True = false;

        int loop_count = 0;
        while (!gamepad1.right_bumper && !gamepad1.left_bumper) {

            if (gamepad1.a) stage1True = true;
            if (gamepad1.b) stage2True = true;
            if (gamepad1.x) stage3True = true;
            if (gamepad1.y) stage4True = true;

            telemetry.addData("Status","Waiting for both BUMPERS");
            telemetry.addData("Stage1", stage1True);
            telemetry.addData("Stage2", stage2True);
            telemetry.addData("Stage3", stage3True);
            telemetry.addData("Stage4", stage4True);
            sleep(10);
            if ((loop_count++ % 100) == 0) {
                DbgLog.msg("Still stuck waiting for double bumper");
            }
        }

        telemetry.addData("Got both BUMPERS","");

        waitForStart();
        telemetry.clearData();
        elapsedTime.reset();

        debugLogMotors();
        raiseArmToStop(0.3);
        if (stage1True) {
            telemetry.addData("Stage1","Started");
            lowerArmDegrees(10, 0.3);
            extendArm(1, 0.5);
            telemetry.addData("Stage1", "Finished");
            sleep(1300);
        }
        if (stage2True) {
            telemetry.addData("Stage2","Started");
            lowerArmDegrees(40, 0.3);
            extendArm(10, 0.5);
            sleep(5000);
            extendArm(-10, 0.5);
            telemetry.addData("Stage2", "Finished");
            sleep(1300);
        }
        if (stage3True) {
            telemetry.addData("Stage3","Started");
            lowerArmDegrees(45, 0.3);
            telemetry.addData("Stage3", "Finished");
            sleep(1300);
        }
        if (stage4True) {
            telemetry.addData("Stage4","Started");
            lowerArmDegrees(-75, 0.3);
            telemetry.addData("Stage4", "Finished");
            sleep(1300);
        }
        telemetry.addData("Z","Finished");

    }
}
