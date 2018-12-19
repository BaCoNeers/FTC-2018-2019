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

package org.firstinspires.ftc.teamcode.Autonomous.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.AutoDrive;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.Task;
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

@Autonomous(name="NewCraterAuto", group="SimonsPlayGround")
public class NewCrater extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    RoverRucusConfiguration config;
    //Task management
    private ArrayList<Task> Tasks = new ArrayList<Task>();

    private AutoDrive Drive;

    private TensorFlowCubeDetection tensorFlow = new TensorFlowCubeDetection();
    Boolean TensorFlowTest = false;

    @Override
    public void init() {
        config = RoverRucusConfiguration.newConfig(hardwareMap,telemetry);

        Drive = new AutoDrive(config,telemetry);
        tensorFlow.Int(telemetry,hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        //Context Forward Turning Strafing
        tensorFlow.start();
        Tasks.add(new Task(true,1f));
        Tasks.add(new Task(50f,0.4f,"Forward"));
        Tasks.add(new Task(false,1f));
        Tasks.add(new Task(tensorFlow));
        Tasks.add(new Task(500,0.4f,"Forward"));
        Tasks.add(new Task(700,1f,"Forward"));


    }


    @Override
    public void loop() {
        Drive.Update(Tasks);

        if(Drive.BoxCheck){
            switch (Drive.BoxPosition){
                case 1:
                    //Tasks.add(1,new Task(-400, 0.5f, "Strafing"));
                    Tasks.add(0,new Task(-30,0.3f,"Turning"));
                    Tasks.add(3,new Task(30,0.3f,"Turning"));
                    Drive.BoxCheck = false;
                    break;
                case 2:
                    Drive.BoxCheck = false;
                    break;
                case 3:
                    //Tasks.add(1,new Task(400, 0.5f, "Strafing"));
                    Tasks.add(0,new Task(30,0.3f,"Turning"));
                    Tasks.add(3,new Task(-30,0.3f,"Turning"));
                    Drive.BoxCheck = false;
                    break;
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Stopped");
    }

}
