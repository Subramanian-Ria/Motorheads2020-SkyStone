package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MA3TestTelemetry", group="MA3Test")
//@Disabled

public class MA3TestTelementry extends OpMode {

    MA3TestHardware robot = new MA3TestHardware();
    private ElapsedTime runtime = new ElapsedTime();
    private final double diameter = 4;
    private final double circ = diameter * Math.PI;
    private final double threshold = .2;

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
        runtime.reset();
        //robot.MA3.resetDeviceConfigurationForOpMode();
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
         getRotations(1, 5);
        /*if(index < 4)
        {
            voltages[index] = robot.MA3.getVoltage();

            index++;
        }
        else
        {
            index = 0;
            voltages[index] = robot.MA3.getVoltage();
            index++;
        }
        voltageChange = Math.abs(currentVoltage - lastVoltage);
        if(voltageChange >= threshold)
        {
            rotations += voltageChange/5;
            dist = (voltageChange/5)*circ;
            totalDist += dist;
        }
        if(runtime.milliseconds() >= 250)
        {
            velocity = totalDist - lastDistV;
            lastDistV = totalDist;
            runtime.reset();
        }


        /*if(Math.abs(robot.MA3.getVoltage() - lastVoltage) > .2)
>>>>>>> a5dd110e34879675463c8f07e581d8316454dad0
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
//        telemetry.addData("MA3", robot.MA3.getVoltage());
//        telemetry.addData("Rotations", rotations);
//        telemetry.addData("Total Distance", totalDist);
//        telemetry.addData("Velocity (in/s)", velocity);
        /*telemetry.update();
        lastVoltage = robot.MA3.getVoltage();*/
    }
    public void getRotations(int dir, double endDistIn)
    {
        double totalDist = 0;
        double lastVolt = 0;
        double currentVolt;
        double changeVolt;
        while(totalDist < endDistIn)
        {
            currentVolt = robot.MA3.getVoltage();
            changeVolt = currentVolt - lastVolt;
            if(Math.abs(changeVolt) > threshold)
            {
                if(dir > 0)
                {
                    if(changeVolt < 0)
                    {
                        changeVolt += 5;
                    }
                    totalDist += (changeVolt/5)*circ;
                }
                else
                {
                    if(changeVolt > 0)
                    {
                        changeVolt-=5;
                    }
                    totalDist += Math.abs(changeVolt/5)*circ;
                }
            }
            lastVolt = currentVolt;
            telemetry.addData("Total Distance", totalDist);
            telemetry.addData("MA3", robot.MA3.getVoltage());
            telemetry.addData("Rotations", totalDist/circ);
            telemetry.update();
        }
    }
}