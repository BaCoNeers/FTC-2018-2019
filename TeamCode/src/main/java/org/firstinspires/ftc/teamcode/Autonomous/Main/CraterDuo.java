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

@Autonomous(name="CraterDuoAuto", group="SimonsPlayGround")
public class CraterDuo extends OpMode {
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

        Drive = new AutoDrive(hardwareMap,telemetry);
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
        Tasks.add(new Task(tensorFlow));
        Tasks.add(new Task(500,0.3f,"Forward"));//0
        Tasks.add(new Task(500,0.3f,"Forward"));//2
        Tasks.add(new Task(-500,0.3f,"Forward"));//3
        Tasks.add(new Task(-90,0.3f,"Turning"));//4
        Tasks.add(new Task(-45,0.3f,"Turning"));//6
        Tasks.add(new Task(-200,0.4f,"Strafing"));//7
        Tasks.add(new Task(100,0.3f,"Strafing"));//8  
    }


    @Override
    public void loop() {
        Drive.Update(Tasks);

        if(Drive.BoxCheck){
            switch (Drive.BoxPosition){
                //left
                case 1:
                    Tasks.add(1,new Task(-380, 0.3f, "Strafing"));
                    Tasks.add(5,new Task(800,0.3f,"Forward"));
                    Tasks.add(9,new Task(-110,0.3f,"Turning"));
                    Tasks.add(10,new Task(800,0.3f,"Forward"));
                    Tasks.add(11,new Task(40,0.3f,"Turning"));
                    Tasks.add(12,new Task(1100,0.3f,"Forward"));
                    Tasks.add(13,new Task(-20,0.3f,"Turning"));
                    Tasks.add(14,new Task(-200,0.3f,"Strafing"));
                    Tasks.add(15,new Task(100,0.3f,"Strafing"));
                    Drive.BoxCheck = false;
                    break;
                case 2:
                    Tasks.add(4,new Task(1200,0.3f,"Forward"));
                    Drive.BoxCheck = false;
                    break;
                    //right
                case 3:
                    Tasks.add(1,new Task(340, 0.3f, "Strafing"));
                    Tasks.add(5,new Task(1000,0.3f,"Forward"));
                    Tasks.add(7,new Task(700,0.3f,"Forward"));
                    Tasks.add(8,new Task(20,0.3f,"Turning"));
                    Tasks.add(9,new Task(600,0.3f,"Forward"));
                    Tasks.add(10,new Task(145,0.3f,"Turning"));
                    Tasks.add(11,new Task(1300,0.5f,"Forward"));
                    Tasks.add(12,new Task(-200,0.4f,"Strafing"));
                    Tasks.add(13,new Task(200,0.3f,"Forward"));

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
