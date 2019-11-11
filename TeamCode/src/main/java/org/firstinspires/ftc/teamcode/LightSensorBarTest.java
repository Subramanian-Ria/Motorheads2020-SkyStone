package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="LightSensorBarTest", group="ServoTest")
//@Disabled

public class LightSensorBarTest extends OpMode {

    LightSensorBarTestHardware robot = new LightSensorBarTestHardware();

    private float drive = .8f;

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
        //
        telemetry.addData("sen1a", robot.sen1.alpha());
        telemetry.addData("sen1r", robot.sen1.red());
        telemetry.addData("sen1g", robot.sen1.green());
        telemetry.addData("sen1b", robot.sen1.blue());
//        telemetry.addData("sen2", robot.sen2.alpha());
//        telemetry.addData("sen3", robot.sen3.alpha());
//        telemetry.addData("dist", robot.dist.getDistance(DistanceUnit.INCH));//if the distance is < .5in then the stone is too close despite being skystone
        //and if a nonskystone is more than 5.5in away it needs to be brought closer
        // if stone is really close then skystone reading are lower across the board than regular stone
//        double average = robot.sen1.alpha()+ robot.sen3.alpha() + robot.sen2.alpha();
//        boolean isSkystone = false;
//        average/=3;
//        telemetry.addData("average", (robot.sen1.alpha() + robot.sen2.alpha() + robot.sen3.alpha())/3);
//        if(average < 150)
//        {
//            isSkystone = true;
//        }
//        telemetry.addData("Is A Skystone", isSkystone);
        telemetry.update();
        //not funny didnt laugh
    }

}