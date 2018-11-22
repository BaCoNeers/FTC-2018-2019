package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;

import java.util.ArrayList;


/**
 * Created by Simon on 27/09/2018.
 */

public class CoordinateDrive{

    public Coordinates Coords = new Coordinates(0,0,0);
    private DcMotor[] Motors = new DcMotor[4];
    private float[] Encoders = new float[4];
<<<<<<< HEAD

    private float RobotCirumfrance = 957.557f;
    private float RobotOneDeg = RobotCirumfrance/360f;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/1440f;
=======
    private float[] MotorPower = new float[4];


    private float Encoder = 1120f;
    private float RobotCirumfrance = 2500f;
    private float RobotOneDeg = RobotCirumfrance/360f;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/Encoder;

    private float MarginOfError = 30;

    private Coordinates Goal = null;

    private float Angle;
    private float Distence;
>>>>>>> master

    private Telemetry tel;

    public CoordinateDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, Telemetry tel){
        Motors[0] = FLM;
        Motors[1] = FRM;
        Motors[2] = BLM;
        Motors[3] = BRM;
        this.tel = tel;
    }

<<<<<<< HEAD

    public boolean Forward(float distance, float power){
        int direction = 1;
        if(direction<0){
            direction = -1;
        }
        if(GetAvaragePosition() < distance){
            Mot
        }
    }



=======
    /*
    //need work
    public boolean SetCoordinate(float x,float y, float Power){
        UpdateTelemetry();
        if(Goal == null){
            Goal = new Coordinates(x,y,0);
            SetAngle();
            SetMagnitude();
        }
        if(Angle-MarginOfError < Coords.Angle && Coords.Angle < Angle+MarginOfError){
            Rotate(Power);
            return false;
        }
        else if(Distence-MarginOfError < Coords.Angle && Coords.Angle < Angle+MarginOfError){
            Forward(Power);
            return false;
        }
        return true;
    }
    */

    public void Update(ArrayList<Task> tasks){
        if(tasks.size()>0){
            if(tasks.get(0).Angle == 0){
                if(Forward(tasks.get(0).Forward,tasks.get(0).Power)){
                    tasks.remove(0);
                }
            }
            if(tasks.get(0).Forward == 0){
                if(Rotate(tasks.get(0).Angle, tasks.get(0).Power)){
                    tasks.remove(0);
                }
            }
        }
    }

    public boolean Rotate(float Angle,float Power){
        int direction = 1;
        if(Distence <0){
            direction = -1;
        }
        if(Math.abs(ConvertToAngle(GetLeftAvaragePosition()))< Math.abs(Angle)){
            MotorPower[0] = Power * (direction);
            MotorPower[1] = Power * (direction * -1);
            MotorPower[2] = Power * (direction);
            MotorPower[3] = Power * (direction * -1);
            UpdateMotor(true);
            UpdateEncoders();
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
        int direction = 1;
        if(Distence<0){
            direction = -1;
        }
        if(Math.abs(ConvertToMM(GetAvaragePosition())) < Math.abs(Distance)){
            MotorPower[0] = Power*(direction);
            MotorPower[1] = Power*(direction);
            MotorPower[2] = Power*(direction);
            MotorPower[3] = Power*(direction);
            UpdateMotor(true);
            UpdateEncoders();
            return false;
        }
        else{
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }




    private void SetAngle() {
        float DisiredRotation = (float)Math.atan2(Goal.x-Coords.x,Goal.y-Coords.y);
        Angle = (float)(DisiredRotation * (180/Math.PI));
    }
    private void SetMagnitude(){
        Distence =  (float) (Math.sqrt(Math.pow(Goal.x-Coords.x,2)+Math.pow(Goal.x-Coords.y,2)))/WheelCount;
    }
>>>>>>> master
    public float GetAvaragePosition(){
        return (Math.abs(Encoders[0])+Math.abs(Encoders[1])+Math.abs(Encoders[2])+Math.abs(Encoders[3]))/4f;
    }
    public float GetLeftAvaragePosition(){
        return (Math.abs(Encoders[0]+Encoders[2])*0.5f);
    }

<<<<<<< HEAD
=======
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

>>>>>>> master
    private void UpdateEncoders(){
        Encoders[0] = Motors[0].getCurrentPosition();
        Encoders[1] = Motors[1].getCurrentPosition();
        Encoders[2] = Motors[2].getCurrentPosition();
        Encoders[3] = Motors[3].getCurrentPosition();
    }

    private void UpdatePosition() {
        Coords.x += GetAvaragePosition()*WheelCount*(float)Math.cos(Coords.Angle);
        Coords.y += GetAvaragePosition()*WheelCount*(float)Math.sin(Coords.Angle);
    }

    private void UpdateRotation(){
        if(Encoders[0] > 0 && Encoders[2] > 0 ){
            Coords.Angle += Math.abs(GetLeftAvaragePosition()*WheelCount*RobotOneDeg);
        }
        else{
            Coords.Angle -= Math.abs(GetLeftAvaragePosition()*WheelCount*RobotOneDeg);
        }
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
<<<<<<< HEAD
=======

    }

    private void UpdateTelemetry(){
        tel.addLine("RobotPosition  X:"+Coords.x+" Y:"+Coords.y+" Angle:"+Coords.Angle);
>>>>>>> master
    }


}
