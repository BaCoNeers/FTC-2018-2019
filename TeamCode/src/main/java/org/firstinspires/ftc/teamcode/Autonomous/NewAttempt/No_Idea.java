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

package org.firstinspires.ftc.teamcode.Autonomous.NewAttempt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

@Autonomous(name="No_Idea", group="Linear Opmode")
public class No_Idea extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //28 pulse per rotation
    //Other robot

    //11.1 PPR
    private float WheelEncoder = 1120f;
    private float RobotCirumfrance = 2500f;
    private float RobotOneDeg = RobotCirumfrance/360f;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/WheelEncoder;


    DcMotor[] Motors = new DcMotor[4];
    float[] Power = new float[4];

    DcMotor prim_lift_motor = null;
    DcMotor Sec_lift_motor = null;
    DcMotor Arm_lift_motor =null;

    //TouchSensor LimitSwitch = null;
    boolean down = true;



    ArrayList<Task> Tasks = new ArrayList<>();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        Motors[0] = hardwareMap.get(DcMotor.class, "front_right_drive");
        Motors[1] = hardwareMap.get(DcMotor.class, "front_left_drive");
        Motors[2]  = hardwareMap.get(DcMotor.class, "rear_right_drive");
        Motors[3] = hardwareMap.get(DcMotor.class, "rear_left_drive");

        prim_lift_motor = hardwareMap.get(DcMotor.class,"prim_lift_motor");
        Sec_lift_motor = hardwareMap.get(DcMotor.class,"sec_lift_motor");
        Arm_lift_motor = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        //LimitSwitch = hardwareMap.get(TouchSensor.class, "LiftSwitch");



        Motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motors[1].setDirection(DcMotor.Direction.REVERSE);
        Motors[3].setDirection(DcMotor.Direction.REVERSE);

        //Context Forward,Turning,Strafing

        Tasks.add(new Task(200f,0.5f,"Forward"));
        Tasks.add(new Task(-200f,0.5f,"Forward"));
        Tasks.add(new Task(1000,0.5f,"Strafing"));
        Tasks.add(new Task(-1000,0.5f,"Strafing"));
        Tasks.add(new Task(90, 0.5f,"Turning"));
        Tasks.add(new Task(-90,0.5f,"Turning"));


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addLine(""+Tasks.size());
            if(Tasks.size()>0){
                if(Tasks.get(0).Distance == 0 && Tasks.get(0).Angle == 0 ){
                    if(Strafe(Tasks.get(0).Strafe,Tasks.get(0).Power)) {
                        Tasks.remove(0);
                        ResetEcnoders();
                        sleep(1000);
                    }
                }
                else if(Tasks.get(0).Angle == 0 && Tasks.get(0).Strafe == 0){
                    if (Forward(Tasks.get(0).Distance, Tasks.get(0).Power)) {
                        Tasks.remove(0);
                        ResetEcnoders();
                        sleep(1000);
                    }
                }
                else if(Tasks.get(0).Distance == 0 && Tasks.get(0).Strafe == 0){
                    if(Turning(Tasks.get(0).Angle,Tasks.get(0).Power)){
                        Tasks.remove(0);
                        ResetEcnoders();
                        sleep(1000);
                    }
                }
                /*
                else if(Tasks.get(0).Lift){
                    if(lift()){
                        sleep(1000);
                    };
                }
                */
            }
            telemetry.addLine("Encoder "+GetAvarageEncoderValue());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    boolean Forward(float mm,float power){
        int direction = 1;
        if(mm < 0){
            direction = -1;
        }
        if(Math.abs(ConvertTomm(GetAvarageEncoderValue())) <= Math.abs(mm))  {
            Power[0] = (power * direction);
            Power[1] = (power * direction);
            Power[2] = (power * direction);
            Power[3] = (power * direction);
            AssignPower();
            return false;
        }
        else{
            ResetPower();
            ResetEcnoders();
            return true;
        }

    }

    boolean Strafe(float ticks,float power){
        int direction = 1;
        if(ticks<0){
            direction = -1;
        }
        if(Math.abs(Motors[0].getCurrentPosition()) <= Math.abs(ticks)){
            Power[0] = (power*direction);
            Power[1] = (-power *direction);
            Power[2] = (-power * direction);
            Power[3] = (power * direction);
            AssignPower();
            return false;
        }
        else {
            ResetPower();
            ResetEcnoders();
            return true;
        }

    }

    boolean Turning(float Angle, float power){
        int direction = 1;
        if(Angle<0){
            direction = -1;
        }
        if(Math.abs(ConvertToAngle(Motors[0].getCurrentPosition())) <= Math.abs(Angle)){
            Power[0] = (power*direction);
            Power[1] = (-power*direction);
            Power[2] = (power*direction);
            Power[3] = (-power*direction);
            AssignPower();
            return false;
        }
        else {
            ResetPower();
            ResetEcnoders();
            return true;
        }
    }

    /*
    boolean lift(){
        if(LimitSwitch.isPressed() && down) {
            prim_lift_motor.setPower(0.5);
            Sec_lift_motor.setPower(0.5);
            return false;
        }
        else if(LimitSwitch.isPressed() && !down){
            prim_lift_motor.setPower(-0.5);
            Sec_lift_motor.setPower(-0.5);
            return false;
        }
        else{
            down = !down;
            return true;
        }
    }
    */

    float ConvertTomm(float Encoder){
        return Encoder*WheelCount;
    }
    float ConvertToEncoder(Float mm){
        return mm*1.26f;
    }
    float ConvertToAngle(float Encoder){
        return (Encoder*WheelCount)/RobotOneDeg;
    }
    float ConvertFromAngle(float angle){
        return (angle*RobotOneDeg)*1.26f;
    }

    float GetAvarageEncoderValue(){
        float Encoder = Motors[0].getCurrentPosition() + Motors[1].getCurrentPosition()
                + Motors[2].getCurrentPosition() + Motors[3].getCurrentPosition();
        return Encoder/4f;
    }

    void AssignPower(){
        for(int i=0;i<Motors.length;i++){
            Motors[i].setPower(Power[i]);
        }
    }

    void ResetPower(){
        for(int i=0;i<Motors.length;i++){
            Motors[i].setPower(0);
            Power[i] = 0;
        }
    }

    void UpdatePower(){
        if((Motors[0].getCurrentPosition() + Motors[1].getCurrentPosition())/2 > 100){

        }
    }

    void ResetEcnoders(){
        Motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



}
