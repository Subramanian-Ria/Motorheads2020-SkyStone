package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Basic4WheelDriveTeleop", group="ServoTest")
//@Disabled

public class Basic4WheelDriveTeleop extends OpMode {

    Basic4WheelDriveHardware robot = new Basic4WheelDriveHardware();

    private float drive = .8f;

    @Override
    public void init() {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
    }

    @Override
    public void loop()
    {

        if(Math.abs(gamepad1.right_stick_x) > .1)
        {
            robot.FLTest.setPower(-gamepad1.right_stick_x);
            robot.FRTest.setPower(gamepad1.right_stick_x);
            robot.BLTest.setPower(-gamepad1.right_stick_x);
            robot.BRTest.setPower(gamepad1.right_stick_x);
        }
        else if(Math.abs(gamepad1.left_stick_y) > .1)
        {
            if(gamepad1.left_bumper && gamepad1.left_stick_y > 0)
            {
                robot.FLTest.setPower(.1);
                robot.FRTest.setPower(.1);
                robot.BLTest.setPower(.1);
                robot.BRTest.setPower(.1);
            }
            else if(gamepad1.left_bumper && gamepad1.left_stick_y < 0)
            {
                robot.FLTest.setPower(-.1);
                robot.FRTest.setPower(-.1);
                robot.BLTest.setPower(-.1);
                robot.BRTest.setPower(-.1);
            }
            else {
                robot.FLTest.setPower(gamepad1.left_stick_y);
                robot.FRTest.setPower(gamepad1.left_stick_y);
                robot.BLTest.setPower(gamepad1.left_stick_y);
                robot.BRTest.setPower(gamepad1.left_stick_y);
            }
        }
        else
        {
            robot.FLTest.setPower(0);
            robot.FRTest.setPower(0);
            robot.BLTest.setPower(0);
            robot.BRTest.setPower(0);
        }
    }
}