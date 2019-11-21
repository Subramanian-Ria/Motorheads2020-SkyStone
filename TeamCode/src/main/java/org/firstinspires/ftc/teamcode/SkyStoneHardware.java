package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SkyStoneHardware
{
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armLift;
    //public DcMotor susan;
    public DcMotor armExt;
    public DistanceSensor distSen;
    public Servo claw;
    public Servo wrist;
    public ColorSensor color1;

    //declaring values for use with encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.19685039;     // For fwiguring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     VERT_INCHES             = 3.0;      //inches to raise verticalarm       TODO: test for real value
    static final double     HORZ_INCHES             = 3.0;      //inches to extend horzizantalarm   TODO: test for real value
    static final double     EN_ARM_SPEED            = 0.2;      //speed of arm movement

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        fLMotor = hwMap.get(DcMotor.class, "fLMotor");
        fRMotor = hwMap.get(DcMotor.class, "fRMotor");
        bRMotor = hwMap.get(DcMotor.class, "bRMotor");
        bLMotor = hwMap.get(DcMotor.class, "bLMotor");
        //susan = hwMap.get(DcMotor.class, "susan");
        armExt = hwMap.get(DcMotor.class, "armExt");
        armLift = hwMap.get(DcMotor.class, "armLift");
        distSen = hwMap.get(DistanceSensor.class, "distSen");
        color1 = hwMap.get(ColorSensor.class, "color1");
        claw = hwMap.get(Servo.class, "claw");
        wrist = hwMap.get(Servo.class, "wrist");

        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //flipped these 4
        //TODO: CHECK
        fLMotor.setDirection(DcMotor.Direction.REVERSE);
        fRMotor.setDirection(DcMotor.Direction.FORWARD);
        bLMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.FORWARD);

        fLMotor.setPower(0);
        fRMotor.setPower(0);
        bLMotor.setPower(0);
        bRMotor.setPower(0);

        armLift.setDirection(DcMotor.Direction.FORWARD);
    }
}
