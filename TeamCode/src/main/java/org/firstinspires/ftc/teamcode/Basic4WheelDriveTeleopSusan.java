package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="SkystoneTeleOp", group="ServoTest")
//@Disabled

public class Basic4WheelDriveTeleopSusan extends OpMode {

    SkyStoneHardware robot = new SkyStoneHardware();

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
            robot.fLMotor.setPower(-gamepad1.right_stick_x);
            robot.fRMotor.setPower(gamepad1.right_stick_x);
            robot.bLMotor.setPower(-gamepad1.right_stick_x);
            robot.bRMotor.setPower(gamepad1.right_stick_x);
        }
        else if(Math.abs(gamepad1.left_stick_y) > .1)
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
        else
        {
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
        }
        if(gamepad1.x && robot.susan.getCurrentPosition() < (2*COUNTS_PER_MOTOR_REV))
        //TODO: see below
        {
            if(robot.susan.getPower() != .45)
            {
                double inc = (.45 - robot.susan.getPower())/200;
                for(int i = 0; i < 200; i++)
                {
                    robot.susan.setPower(robot.susan.getPower() + inc);
                }
            }
            //robot.susan.setPower(.45);
            //robot.servo2.setPosition(0.5);
        }
        else if(gamepad1.y && robot.susan.getCurrentPosition() > (-2*COUNTS_PER_MOTOR_REV))
        //TODO: have it rotate in the opposite direction to reverse encoder count
        {
            if(robot.susan.getPower() != -.45)
            {
                double inc = (-.45 - robot.susan.getPower())/200;
                for(int i = 0; i < 200; i++)
                {
                    robot.susan.setPower(robot.susan.getPower() + inc);
                }
            }
            //robot.susan.setPower(-.45);
            // robot.servo2.setPosition(1);
        }
        else
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
    }
}