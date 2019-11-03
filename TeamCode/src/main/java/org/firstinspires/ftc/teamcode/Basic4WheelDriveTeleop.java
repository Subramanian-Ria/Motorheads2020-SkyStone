package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Basic4WheelDriveTeleop", group="ServoTest")
//@Disabled

public class Basic4WheelDriveTeleop extends OpMode {

    Basic4WheelDriveHardware robot = new Basic4WheelDriveHardware();

    private float drive = .8f;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

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
                robot.FLTest.setPower(.15);
                robot.FRTest.setPower(.15);
                robot.BLTest.setPower(.15);
                robot.BRTest.setPower(.15);
            }
            else if(gamepad1.left_bumper && gamepad1.left_stick_y < 0)
            {
                robot.FLTest.setPower(-.15);
                robot.FRTest.setPower(-.15);
                robot.BLTest.setPower(-.15);
                robot.BRTest.setPower(-.15);
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
        if(gamepad1.x && robot.susanTest.getCurrentPosition() < (2*COUNTS_PER_MOTOR_REV))
        //TODO: see below
        {
            if(robot.susanTest.getPower() != .45)
            {
                double inc = (.45 - robot.susanTest.getPower())/200;
                for(int i = 0; i < 200; i++)
                {
                    robot.susanTest.setPower(robot.susanTest.getPower() + inc);
                }
            }
            //robot.susanTest.setPower(.45);
            //robot.servo2.setPosition(0.5);
        }
        else if(gamepad1.y && robot.susanTest.getCurrentPosition() > (-2*COUNTS_PER_MOTOR_REV))
        //TODO: have it rotate in the opposite direction to reverse encoder count
        {
            if(robot.susanTest.getPower() != -.45)
            {
                double inc = (-.45 - robot.susanTest.getPower())/200;
                for(int i = 0; i < 200; i++)
                {
                    robot.susanTest.setPower(robot.susanTest.getPower() + inc);
                }
            }
            //robot.susanTest.setPower(-.45);
            // robot.servo2.setPosition(1);
        }
        else
        {
            if(robot.susanTest.getPower() != 0)
            {
                double inc = -(robot.susanTest.getPower()/200);
                for(int i = 0; i < 200; i++)
                {
                    robot.susanTest.setPower(robot.susanTest.getPower() + inc);
                }
            }
        }
    }
}