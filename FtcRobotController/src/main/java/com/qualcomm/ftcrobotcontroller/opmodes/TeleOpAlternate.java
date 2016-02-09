package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Roarbots on 11/23/2015.
 */
public class TeleOpAlternate extends TeleOpResQCommonV2 {
    private int numOpLoops = 1;
    private boolean pushOnceRT = false;
    private boolean pushOnceLT = false;
    private boolean pushOnceRB = false;
    private boolean extraWheelOn = false;
    private double arm_wheel_power = 0.0;
    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void loop () {
    /*
     * Gamepad 1
     *
     * Gamepad 1 controls the motors via the left and right sticks(tank drive), and it controls the wrist/claw via the a,b,
     * x, y buttons
     */
        // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right

        double left_drive_power = 0.0;
        double right_drive_power = 0.0;
        double turn = 0.0;
        double throttle = 0.0;
        double arm_angle_power = 0.0;
        double arm_extender_power = 0.0;
        double left_stick = gamepad1.left_stick_y;
        double right_stick = gamepad1.right_stick_x;
        double motorPosition = 0;
        double diffMotor_Pos = .1;

        // clip the right/left values so that the values never exceed +/- 1
        left_drive_power = Range.clip(left_stick, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        right_drive_power = Range.clip(right_stick, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        motorPosition = Range.clip(motorPosition, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);


        // write the values to the motors
        if (left_stick != 0) {
            if (right_stick < 0.0) { //turn left

                right_drive_power = left_stick;
                left_drive_power = left_stick + (right_stick * left_stick);

            } else if (right_stick >= 0.0) { //turn right or do forward/backward movement

                left_drive_power = left_stick;
                right_drive_power = left_stick - (right_stick * left_stick);

            }
        } else { //turn in place

            left_drive_power = right_stick;
            right_drive_power = -1 * left_drive_power;

        }

        //actually set the values to the motors
        setLeftPower(left_drive_power);
        setRightPower(right_drive_power);

        //if the right  trigger is pressed and above .5 then the motors
        //will be set to 100% power and move forward
        if (gamepad1.right_trigger >= .5) {
            arm_angle_power = 1.0;
        } else {
            //if the left trigger is pressed and above .5 then the motors
            //will be set to 100% power and move backwards
            if (gamepad1.left_trigger >= .5) {
                arm_angle_power = -1.0;
            } else {
                arm_angle_power = 0.0;
            }
        }
      //  treadAngleMotor.setPower(arm_angle_power);

        if (gamepad2.right_bumper) {
            arm_wheel_power = 0.0;
        } else if (gamepad2.left_bumper) {
            arm_wheel_power = 1.0;
        }

     //   treadPowerMotor.setPower(arm_wheel_power);

        arm_extender_power = Range.clip(gamepad2.right_stick_y, MOTOR_MIN_RANGE, MOTOR_MAX_RANGE);
        if (Math.abs(arm_extender_power) < 0.1) {
            arm_extender_power = 0.0;
        }
       // setExtenderPower(arm_extender_power);


        telemetry.addData("left motor ", left_drive_power);
        telemetry.addData("right motor ", right_drive_power);
        telemetry.addData("tread wheel power ", arm_wheel_power);
        telemetry.addData("tread wheel angle power ", arm_angle_power);
        telemetry.addData("arm extender power ", arm_extender_power);
    }
}
