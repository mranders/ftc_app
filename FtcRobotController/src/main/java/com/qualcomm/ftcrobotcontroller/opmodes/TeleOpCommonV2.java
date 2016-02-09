package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by Roarbots on 10/18/2015.
 */
public abstract class TeleOpCommonV2 extends TeleOpRoarbotsCommon {

    private double myWheelSize;
    private double driveTrainRatio;

    /*
     * Sets up the parameters of the robot to use in our functions
     *
     * wheelSize = Diameter of the wheel in inches
     */
    public void setupRobotParameters(double myWheelSize, double myDriveTrainRatio) {
        myWheelSize = this.myWheelSize;
        driveTrainRatio = myDriveTrainRatio;
    }

    /*
     * Drive the robot forward or backward for a distance.
     *
     * speed    = 1.0 through -1.0 sets the motor power
     * distance = The distance in inches to travel
     */
    public void drive(double speed, double distance) {
        for (int i = 0; i < leftMotors.size(); i++) {
            leftMotors.get(i).setPower(speed);
        }
        for (int i = 0; i < rightMotors.size(); i++) {
            rightMotors.get(i).setPower(speed);
        }

    }

    /*  Creates a function to define a One Point Turn.
        speed = speed of motors
        angle = angle of turn in degrees
        left = direction: left or right
     */
    public void onePointTurn(double speed, double angle, boolean left) {
        if (left == true) {
            for (int i = 0; i < leftMotors.size(); i++) {
                leftMotors.get(i).setPower(speed);
            }
        } else if (left == false) {
            for (int i = 0; i < rightMotors.size(); i++) {
                rightMotors.get(i).setPower(speed);
            }
        }
    }

    /*  Creates a function that defines a Two Point Turn.
        speed = speed of motors
        angle = angle of turn in degrees
        left = direction: left or right
     */
    public void twoPointTurn(double speed, double angle, boolean left) {
        if (left == true) {
            for (int i = 0; i < leftMotors.size(); i++) {
                leftMotors.get(i).setPower(speed);
            }
            for (int i = 0; i < rightMotors.size(); i++) {
                rightMotors.get(i).setPower(speed * .5);
            }
        } else if (left == false) {
            for (int i = 0; i < leftMotors.size(); i++) {
                leftMotors.get(i).setPower(speed * .5);
            }
            for (int i = 0; i < rightMotors.size(); i++) {
                rightMotors.get(i).setPower(speed);
            }
        }
    }
}
