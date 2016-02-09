package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by Roarbots on 10/18/2015.
 */
public abstract class TeleOpCommon extends TeleOpRoarbotsCommon {

    private double myWheelSize;
    private double driveTrainRatio;
    private List<DcMotor> leftMotors = new ArrayList<DcMotor>();
    private List<DcMotor> rightMotors = new ArrayList<DcMotor>();

    /*
     * Sets up the parameters of the robot to use in our functions
     *
     * wheelSize = Diameter of the wheel in inches
     */
    public void setupRobotParameters(double myWheelSize, double myDriveTrainRatio, List<DcMotor>myLeftMotors, List<DcMotor>myRightMotors, List<DcMotor>myArmMotors) {
        myWheelSize = this.myWheelSize;
        driveTrainRatio = myDriveTrainRatio;
        leftMotors.clear();
        leftMotors.addAll(myLeftMotors);
        rightMotors.clear();
        rightMotors.addAll(myRightMotors);
    }
    /*
     * Drive the robot forward or backward for a distance.
     *
     * speed    = 1.0 through -1.0 sets the motor power
     * distance = The distance in inches to travel
     */
    public void drive(double speed, double distance){
        for(int i = 0;i < leftMotors.size();i++)
        {
            leftMotors.get(i).setPower(speed);
        }
        for(int i = 0;i < rightMotors.size();i++)
        {
            rightMotors.get(i).setPower(speed);
        }
    }

    // Creates a function
    public void onePointTurn(double speed, double angle) {
        for(int i = 0;i < leftMotors.size();i++)
        {
            leftMotors.get(i).setPower(speed);
        }
        for(int i = 0;i < rightMotors.size();i++)
        {
            rightMotors.get(i).setPower(speed);
        }
    }
}
