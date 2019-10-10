package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MA3TestTelemetry", group="MA3Test")
//@Disabled

public class MA3TestTelementry extends OpMode {

    MA3TestHardware robot = new MA3TestHardware();
    private double rotationCount = 0;
    boolean rotated = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double lastVoltage = 0;
    private double lastPosition = 0;
    private  double currentPosition = 0;
    private double velocity = 0;


    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
        runtime.reset();
    }

    @Override
    public void loop()
    {
        if(Math.abs(robot.MA3.getVoltage() - lastVoltage) > .2)
        {
            rotationCount+=Math.abs(robot.MA3.getVoltage()-lastVoltage)/5;
            lastVoltage = robot.MA3.getVoltage();
        }
        if(runtime.milliseconds() >= 250)
        {
            currentPosition = rotationCount * 4 * Math.PI;
            velocity = (currentPosition - lastPosition)/runtime.milliseconds() * 1000;
            lastPosition = currentPosition;
            runtime.reset();
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
        /*if(runtime.milliseconds() == 250)
        {
            currentPosition = rotationCount * 4 * Math.PI;
            velocity = (currentPosition - lastPosition)/runtime.milliseconds();
            lastPosition = currentPosition;
            runtime.reset();
        }*/
        telemetry.addData("MA3", robot.MA3.getVoltage());
        //telemetry.addData("rotated", rotated);
        telemetry.addData("Rotations", rotationCount);
        telemetry.addData("Velocity (in/s)", velocity);
        telemetry.update();
    }


}