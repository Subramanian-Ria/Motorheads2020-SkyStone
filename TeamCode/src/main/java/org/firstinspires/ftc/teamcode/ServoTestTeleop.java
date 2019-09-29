package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="ServotTestTeleop", group="ServoTest")
//@Disabled

public class ServoTestTeleop extends OpMode {

    ServoTestHardware robot = new ServoTestHardware();

    private float drive = .8f;

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
    }

    @Override
    public void loop()
    {

        if(gamepad1.a)
        {
            robot.servo1.setPosition(.5);
            robot.servo2.setPosition(0.5);
        }
        if(gamepad1.b)
        {
            robot.servo1.setPosition(0);
            robot.servo2.setPosition(1);
        }
    }

}