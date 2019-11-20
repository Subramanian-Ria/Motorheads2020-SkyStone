package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="SkystoneTeleOp", group="Skystone")
//@Disabled

public class SkystoneTeleop extends OpMode {

    SkyStoneHardware robot = new SkyStoneHardware();

    static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
    static final double     COUNTS_PER_REV_ARM = 1440;
    static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
    static final double     DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    double susanPower = .45;
    double liftPower = .4;

    @Override
    public void init() {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        if(Math.abs(gamepad1.left_stick_y) > .1)
        {
            if(gamepad1.left_bumper && gamepad1.left_stick_y > 0)
            {
                robot.fLMotor.setPower(.15);
                robot.fRMotor.setPower(.15);
                robot.bLMotor.setPower(.15);
                robot.bRMotor.setPower(.15);
            }
            else if(gamepad1.left_bumper && gamepad1.left_stick_y < 0)
            {
                robot.fLMotor.setPower(-.15);
                robot.fRMotor.setPower(-.15);
                robot.bLMotor.setPower(-.15);
                robot.bRMotor.setPower(-.15);
            }
            else {
                robot.fLMotor.setPower(gamepad1.left_stick_y);
                robot.fRMotor.setPower(gamepad1.left_stick_y);
                robot.bLMotor.setPower(gamepad1.left_stick_y);
                robot.bRMotor.setPower(gamepad1.left_stick_y);
            }
        }
        else if(Math.abs(gamepad1.right_stick_x) > .1)
        {
            robot.fLMotor.setPower(-gamepad1.right_stick_x);
            robot.fRMotor.setPower(gamepad1.right_stick_x);
            robot.bLMotor.setPower(-gamepad1.right_stick_x);
            robot.bRMotor.setPower(gamepad1.right_stick_x);
        }
        else
        {
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
        }
        //TODO: mak
        if(gamepad1.dpad_up && robot.armExt.getCurrentPosition() < 20)
        {
            robot.armLift.setPower(-liftPower);
        }
        else if(gamepad1.dpad_down)
        {
            robot.armLift.setPower(liftPower);
        }
        else
        {
            robot.armLift.setPower(0);
        }
        if(gamepad1.left_trigger >= .5)
        {
            robot.armExt.setPower(.5);
        }
        else if(gamepad1.right_trigger >= .5)
        {
            robot.armExt.setPower(-.5);
        }
        else
        {
            robot.armExt.setPower(0);
        }
        if(gamepad1.x && robot.susan.getCurrentPosition() < (2*COUNTS_PER_REV_ARM))
        //TODO: see below
        {
            susan(0);
        }
        else if(gamepad1.y && robot.susan.getCurrentPosition() > (-2*COUNTS_PER_REV_ARM))
        //TODO: have it rotate in the opposite direction to reverse encoder count
        {
            susan(1);
        }
        else
        {
            susan(2);
        }
        if(gamepad1.a)
        {
            robot.servo1.setPosition(1);
        }
        else if(gamepad1.b)
        {
            robot.servo1.setPosition(0);
        }
    }

    public void susan(int dir)
    {
        //accelerates susan rotation
        if(dir == 0)
        {
            if(robot.susan.getPower() != susanPower)
            {
                double inc = (susanPower - robot.susan.getPower())/200;
                for(int i = 0; i < 200; i++)
                {
                    robot.susan.setPower(robot.susan.getPower() + inc);
                }
            }
        }
        else if (dir == 1)
        {
            if(robot.susan.getPower() != -susanPower)
            {
                double inc = (-susanPower - robot.susan.getPower())/200;
                for(int i = 0; i < 200; i++)
                {
                    robot.susan.setPower(robot.susan.getPower() + inc);
                }
            }
        }
        else if (dir == 2)
        {
            if(robot.susan.getPower() != 0)
            {
                double inc = -(robot.susan.getPower()/200);
                for(int i = 0; i < 200; i++)
                {
                    robot.susan.setPower(robot.susan.getPower() + inc);
                }
            }
        }
        else
        {
            telemetry.addData("not a valid direction", dir );
        }
    }

}