package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="ArmTestTeleop", group="ServoTest")
//@Disabled

public class ArmTestTeleop extends OpMode {

    ArmTestHardware robot = new ArmTestHardware();

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
        telemetry.addData("armEncoder", robot.armTest.getCurrentPosition());
//        if(gamepad1.b)
//        {
//            if(robot.susanTest.getPower() != 0)
//            {
//                double inc = -(robot.susanTest.getPower()/200);
//                for(int i = 0; i < 200; i++)
//                {
//                    robot.susanTest.setPower(robot.susanTest.getPower() + inc);
//                }
//            }
//            //robot.susanTest.setPower(0);
//           // robot.servo2.setPosition(1);
//        }
//        if(gamepad1.x)
//        {
//            if(robot.susanTest.getPower() != .45)
//            {
//                double inc = (.45 - robot.susanTest.getPower())/200;
//                for(int i = 0; i < 200; i++)
//                {
//                    robot.susanTest.setPower(robot.susanTest.getPower() + inc);
//                }
//            }
//            //robot.susanTest.setPower(.45);
//            //robot.servo2.setPosition(0.5);
//        }
//        if(gamepad1.y)
//        {
//            if(robot.susanTest.getPower() != -.45)
//            {
//                double inc = (-.45 - robot.susanTest.getPower())/200;
//                for(int i = 0; i < 200; i++)
//                {
//                    robot.susanTest.setPower(robot.susanTest.getPower() + inc);
//                }
//            }
//            //robot.susanTest.setPower(-.45);
//            // robot.servo2.setPosition(1);
//        }
        if(gamepad1.a)
        {
            robot.armTest.setPower(1);
        }
        else if(gamepad1.b)
        {
            robot.armTest.setPower(-1);
        }
        else if(gamepad1.x)
        {
            robot.armTest.setPower(.7);
        }
        else if(gamepad1.y)
        {
            robot.armTest.setPower(-.7);
        }
        else
        {
            robot.armTest.setPower(0);
        }
        telemetry.update();
    }

}