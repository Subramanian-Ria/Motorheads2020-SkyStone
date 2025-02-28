package legacy;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MecanumHardware3
{
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;

    public DcMotor armFlip;
    public DcMotor elevator;
    public DcMotor armEx;

    public DcMotor bucket;

    public DistanceSensor sensorDist;
    public DistanceSensor sensorDistDepo;
    public DistanceSensor sensorDistElevator;

    public Servo lights;
    public CRServo marker;

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
        fLMotor = hwMap.get(DcMotor.class, "fLMotor");
        fRMotor = hwMap.get(DcMotor.class, "fRMotor");
        bRMotor = hwMap.get(DcMotor.class, "bRMotor");
        bLMotor = hwMap.get(DcMotor.class, "bLMotor");

        lights = hwMap.get(Servo.class, "lights");
        marker = hwMap.get(CRServo.class, "marker");

        armEx = hwMap.get(DcMotor.class, "armEx");
        armFlip = hwMap.get(DcMotor.class, "armFlip");
        elevator = hwMap.get(DcMotor.class, "elevator");

        bucket = hwMap.get(DcMotor.class, "bucket");

        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);

        armEx.setPower(0);
        armFlip.setPower(0);
        elevator.setPower(0);

        bucket.setPower(0);

        lights.setPosition(500);
        marker.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //flipped these 4
        fLMotor.setDirection(DcMotor.Direction.REVERSE);
        fRMotor.setDirection(DcMotor.Direction.FORWARD);
        bLMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.FORWARD);

        armEx.setDirection(DcMotor.Direction.FORWARD);
        armFlip.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.REVERSE);

        bucket.setDirection(DcMotor.Direction.REVERSE);
        marker.setDirection(CRServo.Direction.FORWARD);

        sensorDist = hwMap.get(DistanceSensor.class, "sensorDist");
        sensorDistDepo = hwMap.get(DistanceSensor.class, "sensorDistDepo");
        sensorDistElevator = hwMap.get(DistanceSensor.class, "sensorDistElevator");
    }
}
