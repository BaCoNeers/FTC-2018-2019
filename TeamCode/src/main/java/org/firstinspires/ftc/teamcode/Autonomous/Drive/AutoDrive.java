package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


/**
 * Created by Simon on 27/09/2018.
 */

public class AutoDrive {

    public Coordinates Coords = new Coordinates(0,0,0);
    private DcMotor[] Motors = new DcMotor[4];
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];


    private float Encoder = 1120f;
    private float RobotCirumfrance = 1517.39f;
    private float RobotOneDeg = RobotCirumfrance/360f;
    private float WheelCirumfrance = 320;
    private float WheelCount = WheelCirumfrance/Encoder;

    private float MarginOfError = 30;

    private Telemetry tel;

    public AutoDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, Telemetry tel){
        Motors[0] = FLM;
        Motors[1] = FRM;
        Motors[2] = BLM;
        Motors[3] = BRM;
        this.tel = tel;
    }

    public void Update(ArrayList<Task> tasks){
        UpdateTelemetry();
        UpdateEncoders();
        if(tasks.size()>0){
            if(tasks.get(0).CheckTask()) {
                switch (tasks.get(0).Context) {
                    case "Forward":
                        if (Forward(tasks.get(0).Value, tasks.get(0).Power)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "Turning":
                        if (Rotate(tasks.get(0).Value, tasks.get(0).Power)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "Strafing":
                        if (Strafe(tasks.get(0).Value, tasks.get(0).Power)) {
                            tasks.remove(0);
                            return;
                        }
                        break;

                }
            }
        }
    }

    public boolean Rotate(float Angle,float Power){
        if(Math.abs(ConvertToAngle(GetAvarage()))< Math.abs(Angle)){
            MotorPower[0] = Power;
            MotorPower[1] = -Power;
            MotorPower[2] = Power;
            MotorPower[3] = -Power;
            UpdateMotor(true);
            tel.addLine("Turning Running");
            return false;
        }
        else{
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }

    public boolean Forward(float Distance,float Power){
        if(Math.abs(ConvertToMM(GetAvarage())) < Math.abs(Distance)){
            MotorPower[0] = Power;
            MotorPower[1] = Power;
            MotorPower[2] = Power;
            MotorPower[3] = Power;
            UpdateMotor(true);
            tel.addLine("Forward Running");
            return false;
        }
        else{
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }

    public boolean Strafe(float Distance, float Power){
        if(Math.abs(ConvertToMM(GetAvarage()))<Distance){
            MotorPower[0] = Power;
            MotorPower[1] = -Power;
            MotorPower[2] = -Power;
            MotorPower[3] = Power;
            UpdateMotor(true);
            return false;
        }
        else{
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }

    public float GetAvarage(){
        float value = 0;
        for(int i=0;i<Encoders.length;i++){
            value+= Math.abs(Encoders[i]);
        }
        return value/Encoders.length;
    }

    public float ConvertToMM(float Encoder){
        return Encoder*WheelCount;
    }
    public float ConvertToAngle(float Encoder){
        return (Encoder*WheelCount)/RobotOneDeg;
    }

    private void UpdateMotor(boolean On){
        if(On){
            Motors[0].setPower(MotorPower[0]);
            Motors[1].setPower(MotorPower[1]);
            Motors[2].setPower(MotorPower[2]);
            Motors[3].setPower(MotorPower[3]);
        }
        else {
            Motors[0].setPower(0);
            Motors[1].setPower(0);
            Motors[2].setPower(0);
            Motors[3].setPower(0);
        }
    }

    private void UpdateEncoders(){
        Encoders[0] = Motors[0].getCurrentPosition();
        Encoders[1] = Motors[1].getCurrentPosition();
        Encoders[2] = Motors[2].getCurrentPosition();
        Encoders[3] = Motors[3].getCurrentPosition();
    }


    private void ResestMotors(){
        Motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void UpdateTelemetry(){
        tel.addLine("Avg Encoder: "+GetAvarage());
    }



}
