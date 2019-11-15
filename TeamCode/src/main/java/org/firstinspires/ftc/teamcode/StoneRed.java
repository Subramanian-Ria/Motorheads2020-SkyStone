package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="StoneRed", group="Skystone")
public class StoneRed extends LinearOpMode {

    /* Declare OpMode members. */
    SkyStoneHardware robot = new SkyStoneHardware();
    private ElapsedTime runtime = new ElapsedTime();
    String xyz = "z";

    static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
    static final double     COUNTS_PER_REV_ARM = 1495; //Tetrix TorqueNado 60:1 - changing to hd hex motor - think we want the value below,2240 switching because it
    //TODO: ALRIGHT LISTEN HERE - APPARENTLY FOR NO APPARENT REASON THAT HAS ANY APPARENT REASON TO APPARENTLY WORK YOU HAVE TO STOP AND RESET THE ENCODER AFTER YOU USE THE MOTOR WHILE ALSO STOPPING AND RESETTING THE ENCODER AT THE START OF THE PROGRAM
//    static final double     COUNTS_PER_REV_ARM_LIFT = 2240; //rev core hex motor -- value is irrelevant -- core hex motor is 288
    static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
//    static final double     COUNTS_PER_INCH_ARM_LIFT = COUNTS_PER_REV_ARM_LIFT/4;
    static final double     DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP TODO: TEST THIS
    static final double     WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //init distance sensors
        /*
        Rev2mDistanceSensor sensorRangeR = hardwareMap.get(Rev2mDistanceSensor.class, "sensorRangeR");
        Rev2mDistanceSensor sensorRangeL = hardwareMap.get(Rev2mDistanceSensor.class, "sensorRangeL");
        sensorRangeL.initialize();
        sensorRangeR.initialize();
        */
        //side motors
        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        /*
        i'm almost certain that what I wrote below is wrong, and has a lot of issues. Feel free to scold me, so long as you get it working
        and explain what you changed and why in commit messages. Hopefully what I wrote was decent enough for only a few lines of change. Yay is suck - Xander
         */
        /*AUTONOMOUS MEASUREMENTS/SEQUENCE
        from start pos(center aligned with in between 2 and 3
        move 3.5in backwards(to scan for first skystone
        move 93in forward (assuming skystone is at 3) (at like 77in drop skystone)
        extend arm 38.25in (probably)
        retract the 37.25 (to avoid nudging it out of validity
        move 113in
        retract 1 in (to get it back to flush
         */
//the below assumes that first block is skystone. add addtional movement at start and scan code for final
//        encoderCorrectionDrive(-3.5, 15,1);
//        encoderCorrectionDrive(93, 15, 1);
//        armLift(1,.5,5);
        armExtend(40,1,10);
        robot.armExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armExtend(40,1, 10);
//        sleep(5000);//part where the claw scans and grabs block
//        armExtend(40, -1, 10);//retracts arm back to robot
//        encoderDrive(40, "f", 15, 1);//drives down to field
//        armExtend(20,1,10);
//        armExtend(20,-1,10);
//        encoderDrive(50, "b", 15, 1);
//        armExtend(40,1, 10);
//        sleep(5000);
//        armExtend(40,-1,10);
//        encoderDrive(45, "f", 15, 1);
//        //susanrotate
//        armExtend(5,1,10);
//        //claw release
//        armExtend(5, -1, 10);
//        encoderDrive(20, "b", 15, 1);
    }

    public static double counts(double inches)
    {
        double newInches = (inches - 3.7959) / 1.1239;
        return newInches;
    }



    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setPower(lpower);
            robot.fRMotor.setPower(rpower);
            robot.bLMotor.setPower(lpower);
            robot.bRMotor.setPower(rpower);
        }
    }

    public void armExtend(double inches, double topPower, double timeoutS)
    {
        //TODO: CHECK PULLEY CIRCUMFERENCE
        int target = robot.armExt.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_ARM);

        robot.armExt.setTargetPosition(target);
        robot.armExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && robot.armExt.isBusy())
        {
            double error = target - robot.armExt.getCurrentPosition();
            //TODO: CHECK PID MULTIPLIER FOR ARM EX (CURRENTLY USING SAME C AS DRIVING
            double power = topPower * pidMultiplierDriving(error);
            robot.armExt.setPower(power);
            telemetry.addData("Path1",  "Running to %7.2f", inches);
            //TODO: the below equation is probably wrong
            telemetry.addData("Path2",  "Running at %7.2f", robot.armExt.getCurrentPosition() / (int)(inches * COUNTS_PER_INCH_ARM));
            telemetry.update();
        }
        robot.armExt.setPower(0);
        robot.armExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    public void armLift(double inches, double topPower, double timeoutS)
//    {
//        //TODO: CHECK PULLEY CIRCUMFERENCE
//        int target = robot.armLift.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_ARM_LIFT);
//
//        robot.armLift.setTargetPosition(target);
//        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//        runtime.reset();
//        while(opModeIsActive() && runtime.seconds() < timeoutS && robot.armLift.isBusy())
//        {
//            double error = target - robot.armLift.getCurrentPosition();
//            //TODO: CHECK PID MULTIPLIER FOR ARM EX (CURRENTLY USING SAME C AS DRIVING
//            double power = topPower * pidMultiplierDriving(error);
//            robot.armLift.setPower(power);
//            telemetry.addData("Path1",  "Running to %7d", target);
//            telemetry.addData("Path2",  "Running at %7d", robot.armLift.getCurrentPosition());
//            telemetry.update();
//        }
//        robot.armLift.setPower(0);
//    }

    public void turnDegrees(double degrees, String xyz, double topPower, double timeoutS, boolean isCorrection)
    {
        //TODO: TEST
        degrees *= -1;
        double originalAngle = readAngle(xyz);


        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double target =angle + degrees;
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            if(!isCorrection)
            {
                powerScaled = topPower * (error / 180) * pidMultiplierTurning(error);
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            }
        } while (opModeIsActive() && ((error > .3) || (error < -0.3)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);
    }

    public void turnToPosition (double target, String xyz, double topPower, double timeoutS, boolean isCorrection) {
        target*= -1;
        double originalAngle = readAngle(xyz);


        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            if (!isCorrection) {
                powerScaled = topPower * (error / 180) * pidMultiplierTurning(error);
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            }
        } while (opModeIsActive() && ((error > .3) || (error < -0.3)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

    public double pidMultiplierDriving(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 100;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }
    public double pidMultiplierTurning(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        double C = .1;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }

    public double readAngle(String xyz) {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (xyz.equals("x")) {
            return angles.thirdAngle;
        } else if (xyz.equals("y")) {
            return angles.secondAngle;
        } else if (xyz.equals("z")) {
            return angles.firstAngle;
        } else {
            return 0;
        }
    }

    public void encoderCorrectionDrive(double inches, double timeoutS, double topPower)
    {
        //TODO: FIGURE WTF IS UP W/MOTOR DIRECTIONS/HEADINGS
        inches *= -1;
        int TargetFL = 0;
        int TargetFR = 0;
        int TargetBL = 0;
        int TargetBR = 0;
        double errorFL = 0;
        double errorFR = 0;
        double errorBL = 0;
        double errorBR = 0;
        double powerFL = 0;
        double powerFR = 0;
        double powerBL = 0;
        double powerBR = 0;


        //String heading = direction;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
            TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
            TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
            TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);



            // Determine new target position, and pass to motor controller

            robot.fLMotor.setTargetPosition(TargetFL);
            robot.fRMotor.setTargetPosition(TargetFR);
            robot.bRMotor.setTargetPosition(TargetBR);
            robot.bLMotor.setTargetPosition(TargetBL);


            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //CORRECTS POWER BASED ON DISTANCE FROM TARGET TO PREVENT OVERSHOOT AND MOTOR INCONSISTENCY
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
            {
                errorFL = TargetFL - robot.fLMotor.getCurrentPosition();
                errorFR = TargetFR - robot.fRMotor.getCurrentPosition();
                errorBL = TargetBL - robot.bLMotor.getCurrentPosition();
                errorBR = TargetBR - robot.bRMotor.getCurrentPosition();

                powerFL = topPower * pidMultiplierDriving(errorFL);
                powerFR = topPower * pidMultiplierDriving(errorFR);
                powerBL = topPower * pidMultiplierDriving(errorBL);
                powerBR = topPower* pidMultiplierDriving(errorBR);

                robot.fLMotor.setPower(Math.abs(powerFL));
                robot.fRMotor.setPower(Math.abs(powerFR));
                robot.bRMotor.setPower(Math.abs(powerBL));
                robot.bLMotor.setPower(Math.abs(powerBR));
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                telemetry.update();
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITION and STOP_AND_RESET_ENCODER
            robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
    {
        //TODO: FIGURE WTF IS UP W/MOTOR DIRECTIONS/HEADINGS
        int TargetFL = 0;
        int TargetFR = 0;
        int TargetBL = 0;
        int TargetBR = 0;


        String heading = direction;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if(heading == "f")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

            }

            else if(heading == "b")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);


            }

            else
            {
                telemetry.addData("not a valid direction", heading );
            }

            // Determine new target position, and pass to motor controller

            robot.fLMotor.setTargetPosition(TargetFL);
            robot.fRMotor.setTargetPosition(TargetFR);
            robot.bRMotor.setTargetPosition(TargetBR);
            robot.bLMotor.setTargetPosition(TargetBL);


            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fLMotor.setPower(Math.abs(Speed));
            robot.fRMotor.setPower(Math.abs(Speed));
            robot.bRMotor.setPower(Math.abs(Speed));
            robot.bLMotor.setPower(Math.abs(Speed));
            /*robot.fLMotor.setPower(Speed);
            robot.fRMotor.setPower(Speed);
            robot.bRMotor.setPower(Speed);
            robot.bLMotor.setPower(Speed);*/


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
            {

                //Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                telemetry.update();
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}