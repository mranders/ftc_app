package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Roarbots on 12/13/2015.
 */
public class Test extends OpMode {
    String servo1 = "servo1";
    public Servo Tester;


    @Override
    public void init(){
        Tester = hardwareMap.servo.get(servo1);

    }

    @Override
    public void loop(){

        if (gamepad1.x) {
            Tester.setPosition(1);
        } else if (gamepad1.b) {
            Tester.setPosition(0);
        }
    }
}
