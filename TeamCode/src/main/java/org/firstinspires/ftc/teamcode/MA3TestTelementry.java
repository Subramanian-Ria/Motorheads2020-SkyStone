package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MA3TestTelemetry", group="MA3Test")
//@Disabled

public class MA3TestTelementry extends OpMode {

    MA3TestHardware robot = new MA3TestHardware();
    private final double Cir = 4 * Math.PI;
    private double rotations = 0;
    private int rotationCount = 0;
    int positiveDirection = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private double lastVoltage = 0;
    private double startingVoltage = 0;
    private double lastPosition = 0;
    private  double currentPosition = 0;
    private double velocity = 0;
    private int count = 0;
    private double distance = 0;


    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
        runtime.reset();
        //robot.MA3.resetDeviceConfigurationForOpMode();
        startingVoltage = robot.MA3.getVoltage();
        lastVoltage = startingVoltage;
    }

    @Override
    public void loop()
    {
//        if(robot.MA3.getVoltage() > startingVoltage - .05 && robot.MA3.getVoltage() < startingVoltage + .05 && !(robot.MA3.getVoltage() == startingVoltage))
//        {
//            rotations++;
//        }
//        else
//        {
//            rotations += (robot.MA3.getVoltage() - lastVoltage) / 5;
//        }
//        if(runtime.milliseconds() >= 250)
//        {
//            velocity = (currentPosition - lastPosition)/runtime.milliseconds() * 1000;
//            distance = rotations * Cir;
//            lastVoltage = robot.MA3.getVoltage();
//            runtime.reset();
//        }

        if(robot.MA3.getVoltage() - lastVoltage > 4.85)
        {
            positiveDirection = -1;
        }
        else if(robot.MA3.getVoltage() - lastVoltage > 0)
        {
            positiveDirection = 1;
        }
        else if(robot.MA3.getVoltage() - lastVoltage < -4.85)
        {
            positiveDirection = 1;
        }
        else if(robot.MA3.getVoltage() - lastVoltage < 0)
        {
            positiveDirection = -1;
        }
        else
        {
            positiveDirection = 0;
        }
//        if(Math.abs(robot.MA3.getVoltage() - startingVoltage) > 4.95 || Math.abs(robot.MA3.getVoltage() - startingVoltage) < .05)
//        {
//            rotationCount++;
//        }
        if(positiveDirection == 1)
        {
            if(robot.MA3.getVoltage() < startingVoltage)
            {
                rotations = rotationCount + (5 - Math.abs(robot.MA3.getVoltage() - startingVoltage));
            }
            else
            {
                rotations = rotationCount + robot.MA3.getVoltage()-startingVoltage;
            }
        }
        else
        {
            if(robot.MA3.getVoltage() > startingVoltage)
            {
                rotations = rotationCount + (5 - Math.abs(robot.MA3.getVoltage() - startingVoltage));
            }
            else
            {
                rotations = Math.abs(rotationCount + robot.MA3.getVoltage()-startingVoltage);
            }
        }
        /*if((robot.MA3.getVoltage() > robot.MA3.getMaxVoltage() - .1 || robot.MA3.getVoltage() < .1) && rotated == false)
        {
            rotationCount++;
            rotated = true;
        }
        if(rotated == true && robot.MA3.getVoltage() > .1 && robot.MA3.getVoltage() < .5)
        {
            rotated = false;
        }*/
        //rotations = rotationCount + robot.MA3.getVoltage()/5;
        /*if(count == 0)
        {
            rotations = count;
            count++;
        }*/
        if(runtime.milliseconds() >= 250)
        {
            currentPosition = rotations * 4 * Math.PI;
            velocity = (currentPosition - lastPosition)/runtime.milliseconds() * 1000;
            lastPosition = currentPosition;
            runtime.reset();
        }
        distance = rotations * 4 * Math.PI;

        telemetry.addData("starting Voltage", startingVoltage);
        telemetry.addData("Positive Direction", positiveDirection);
        telemetry.addData("MA3", robot.MA3.getVoltage());
        telemetry.addData("Dist", distance);
        telemetry.addData("Rotations", rotations);
        telemetry.addData("Rotation Count", rotationCount);
        telemetry.addData("Velocity (in/s)", velocity);
        telemetry.update();
        lastVoltage = robot.MA3.getVoltage();
        //rotations = 0;
    }

}