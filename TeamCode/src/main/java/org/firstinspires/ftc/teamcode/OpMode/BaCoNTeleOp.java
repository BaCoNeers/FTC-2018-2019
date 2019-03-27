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

package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;
import org.firstinspires.ftc.teamcode.common.BaconOpMode;

/**
<<<<<<< HEAD
 * Main TeleOp Class
 * This is TeleOp references other classes related to TeleOp.
=======
 * Crater Teleop Class
>>>>>>> autonomous
 */

@TeleOp
public class BaCoNTeleOp extends BaconOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public RoverRucusConfiguration config;
    public Drive drive;
    public Lift lift;
    public ArmLift armLift;

    @Override
    protected void onInit(){
        config = RoverRucusConfiguration.newConfig(hardwareMap, telemetry);
        drive = new Drive(this, config);
        lift = new Lift(this, config);
        //armLift = new ArmLift(this, config);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void activeLoop(){
        drive.updateDrive();
        lift.updateLift();
        armLift.updateArmLift();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

}
