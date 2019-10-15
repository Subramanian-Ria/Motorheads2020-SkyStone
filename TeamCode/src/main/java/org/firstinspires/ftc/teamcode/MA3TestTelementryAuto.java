/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="MA3TestTelemetryAuton", group="MA3Test")
//@Disabled
public class MA3TestTelementryAuto extends LinearOpMode {

    MA3TestHardware robot   = new MA3TestHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private final double diameter = 4;
    private final double circ = diameter * Math.PI;
    private final double threshold = .07;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        robot.fLMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.fRMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.bLMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.bRMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        robot.fLMotor.setPower(.5);
        robot.fRMotor.setPower(.5);
        robot.bLMotor.setPower(.5);
        robot.bRMotor.setPower(.5);
        getRotations(circ*5, 15);
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
        telemetry.addData("Path", "Complete");
        sleep(500);
    }

    public void getRotations(double endDistIn, double timeoutS)
    {
        if(opModeIsActive())
        {
            runtime.reset();
            double totalDist = 0;
            double lastVolt = 0;
            double currentVolt;
            double changeVolt;
            while(totalDist < endDistIn && opModeIsActive() && runtime.seconds() < timeoutS)
            {
                currentVolt = robot.MA3.getVoltage();
                changeVolt = currentVolt - lastVolt;
                if(Math.abs(changeVolt) > threshold)
                {
                    if(changeVolt < 0)
                    {
                        changeVolt += 5;
                    }
                    totalDist += (changeVolt/5)*circ;
                }
                lastVolt = currentVolt;
                telemetry.addData("MA3", currentVolt);
                telemetry.addData("Time", runtime.seconds());
                telemetry.addData("Change Volt", changeVolt);
                telemetry.addData("Total Distance", totalDist);
                telemetry.addData("Remaining Distance", endDistIn - totalDist);
                telemetry.addData("Rotations", totalDist/circ);
                telemetry.update();
            }
        }
    }

    public void getRotations(int dir, double endDistIn, double timeoutS)
    {
        if(opModeIsActive())
        {
            runtime.reset();
            double totalDist = 0;
            double lastVolt = 0;
            double currentVolt;
            double changeVolt;
            while(totalDist < endDistIn && opModeIsActive() && runtime.seconds() < timeoutS)
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
                telemetry.addData("MA3", robot.MA3.getVoltage());
                telemetry.addData("Time", runtime.seconds());
                telemetry.addData("Total Distance", totalDist);
                telemetry.addData("Remaining Distance", endDistIn - totalDist);
                telemetry.addData("Rotations", totalDist/circ);
                telemetry.update();
            }
        }

    }
}
