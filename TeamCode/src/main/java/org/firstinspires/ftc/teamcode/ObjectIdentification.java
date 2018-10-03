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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name="Vuforia test", group="Vuforia")
public class ObjectIdentification extends LinearOpMode {

    VuforiaLocalizer vuforia;
   //VuforiaLocalizer vuforia2;
    VuforiaLocalizer vuforia3;

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaNavigationWebcam vuforiaNavigation = new VuforiaNavigationWebcam();
    //private VuforiaNavigationWebcam vuforiaNavigation2 = new VuforiaNavigationWebcam();
    //private VuforiaNavigationWebcam vuforiaNavigation3 = new VuforiaNavigationWebcam();

    @Override public void runOpMode() {

        vuforiaNavigation.intVuforia(vuforia,hardwareMap,telemetry,"Webcam 1",0,0,0);
       // vuforiaNavigation2.intVuforia(vuforia2,hardwareMap,telemetry,"Webcam 2",0,0,0);
        //vuforiaNavigation3.intVuforia(vuforia3,hardwareMap,telemetry,"Webcam 3",0,0,0);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            vuforiaNavigation.updateVuforia();
            //vuforiaNavigation2.updateVuforia();
            //vuforiaNavigation3.updateVuforia();
        }
    }
}
