package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="MA3TestTelemetry", group="MA3Test")
//@Disabled

public class MA3TestTelementry extends OpMode {

    MA3TestHardware robot = new MA3TestHardware();
    double rotationCount = 0;
    boolean rotated = false;


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
        if((robot.MA3.getVoltage() > robot.MA3.getMaxVoltage() - .1 || robot.MA3.getVoltage() < .1) && rotated == false)
        {
            rotationCount++;
            rotated = true;
        }
        if(rotated == true && robot.MA3.getVoltage() > .1 && robot.MA3.getVoltage() < .5)
        {
            rotated = false;
        }
        telemetry.addData("MA3", robot.MA3.getVoltage());
        telemetry.addData("rotated", rotated);
        telemetry.addData("Rotations", rotationCount);
        telemetry.update();
    }


}