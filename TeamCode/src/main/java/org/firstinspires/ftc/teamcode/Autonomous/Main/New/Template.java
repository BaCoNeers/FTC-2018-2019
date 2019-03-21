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

package org.firstinspires.ftc.teamcode.Autonomous.Main.New;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.NewAutoDrive;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.ForwardTask;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.LiftTask;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.MainTask;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.TensorFlow;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.TurningTask;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.New.Wait;
import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;
import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

import java.util.ArrayList;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Template", group="SimonsPlayGround")
public class Template extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private RoverRucusConfiguration config;
    //Task management
    private NewAutoDrive Drive;

    ArrayList<MainTask> left = new ArrayList<>();
    ArrayList<MainTask> middle = new ArrayList<>();
    ArrayList<MainTask> right = new ArrayList<>();


    @Override
    public void init() {
        config = RoverRucusConfiguration.newConfig(hardwareMap,telemetry);

        Drive = new NewAutoDrive(config,320,1120,telemetry,hardwareMap);

        Drive.InitialiseIMU(hardwareMap);
        while (Drive.CalabrateIMU());


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addLine("Starting");



        //Find the position of the cube. This should be run when the camera can see
        //all three objects. Unless it cant find all three objects it will defualt to the middle position
        Drive.CubePosition();

        //Forward movement is mesaured in mm
        //turning is measured in degrees

        //left
        left.add(new TurningTask(0.3f,30));
        left.add(new ForwardTask(0.3f,200));


        //middle
        middle.add(new ForwardTask(0.3f,200));

        //right
        right.add(new TurningTask(0.3f,-30));
        right.add(new ForwardTask(0.3f,200));


        Drive.Tasks.add(new TensorFlow(left,middle,right));
        telemetry.update();
    }


    @Override
    public void loop() {
        Drive.Update();
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Stopped");
    }

}
