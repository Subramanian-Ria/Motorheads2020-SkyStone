package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmTestHardware
{

    //public DistanceSensor distSen;
    public DcMotor armTest;
   // public Servo servo2;

    //declaring values for use with encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.19685039;     // For figuring circumference
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
        armTest = hwMap.get(DcMotor.class, "armTest");
        armTest.setDirection(DcMotorSimple.Direction.FORWARD);
        //servo2 = hwMap.get(Servo.class, "servo2");

    }
}
