package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;


/**
 * Created by Simon on 27/09/2018.
 */

public class CoordinateDrive{

    public Coordinates Coords = new Coordinates(0,0,0);
    private DcMotor[] Motors = new DcMotor[4];
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];
    private float RobotCirumfrance = 957.557f;
    private float RobotOneDeg = RobotCirumfrance/360f;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/1440f;
    private Coordinates Goal = null;
    private float Angle;
    private float Distence;
    private Telemetry tel;
    private int count = 0;

    public CoordinateDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, Telemetry tel){
        Motors[0] = FLM;
        Motors[1] = FRM;
        Motors[2] = BLM;
        Motors[3] = BRM;
        this.tel = tel;
    }

    //need work
    public boolean SetCoordinate(float x,float y, float Power){
        if(Goal == null){
            Goal = new Coordinates(x,y,0);
            SetAngle();
            SetMagnitude();
            SetUpRotation(Power);
            tel.addData("Setup","Complete");
        }
        if(CheckRotation()){
            SetUpRotation(Power);
            tel.addLine("Rotation");
        }
        else{
            if(CheckMovement()) {
                SetUpMovement(Power);
                tel.addLine("Movement");
            }
            else{
                Goal = null;
                return true;
            }
        }
        return false;

    }

    private void SetUpRotation(float Power){
        int direction = 1;
        ResestMotors();
        if (Angle < 0) {
            direction = -1;
        }
        MotorPower[0] = Power * (direction);
        MotorPower[1] = Power * (direction * -1);
        MotorPower[2] = Power * (direction);
        MotorPower[3] = Power * (direction * -1);
        count++;
        tel.addLine("Set up motor "+count);
    }
    private boolean CheckRotation(){
        UpdateEncoders();
        tel.addLine("Avarage position"+GetLeftAvaragePosition());
        if ((GetLeftAvaragePosition())<2000){
            UpdateEncoders();
            UpdateMotor(true);
            //UpdatePowers();
            return true;
        }
        else {
            UpdateMotor(false);
            UpdateRotation();
            ResestMotors();
            return false;
        }
    }

    private void SetUpMovement(float Power){
        ResestMotors();
        int direction = 1;
        if(Distence<0){
            direction = -1;
        }
        MotorPower[0] = Power*(direction);
        MotorPower[1] = Power*(direction);
        MotorPower[2] = Power*(direction);
        MotorPower[3] = Power*(direction);
    }
    private boolean CheckMovement(){
        UpdateEncoders();
        if (GetAvaragePosition()<Math.abs(Distence)){
            UpdateEncoders();
            UpdateMotor(true);
            //UpdatePowers();
            return true;
        }
        else {
            UpdateMotor(false);
            UpdatePosition();
            ResestMotors();
            return false;
        }
    }


    private void SetAngle() {
        float DisiredRotation = (float)Math.atan2(Goal.x-Coords.x,Goal.y-Coords.y);
        Angle = (float)(DisiredRotation * (180/Math.PI));
    }
    private void SetMagnitude(){
        Distence =  (float) (Math.sqrt(Math.pow(Goal.x-Coords.x,2)+Math.pow(Goal.x-Coords.y,2)))/WheelCount;
    };
    public float GetAvaragePosition(){
        return (Math.abs(Encoders[0])+Math.abs(Encoders[1])+Math.abs(Encoders[2])+Math.abs(Encoders[3]))/4f;
    }
    public float GetLeftAvaragePosition(){
        return (Math.abs(Encoders[0]+Encoders[2])*0.5f);
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

    private void UpdatePowers() {
        MotorPower[0] = MotorPower[0]*(((MotorPower[1]+MotorPower[2]+MotorPower[3])/3)/MotorPower[0]);
        MotorPower[1] = MotorPower[1]*(((MotorPower[0]+MotorPower[2]+MotorPower[3])/3)/MotorPower[1]);
        MotorPower[2] = MotorPower[2]*(((MotorPower[0]+MotorPower[1]+MotorPower[3])/3)/MotorPower[2]);
        MotorPower[3] = MotorPower[3]*(((MotorPower[0]+MotorPower[1]+MotorPower[2])/3)/MotorPower[3]);
    }

    private void UpdatePosition() {
        Coords.x += GetAvaragePosition()*WheelCount*(float)Math.cos(Coords.Angle);
        Coords.y += GetAvaragePosition()*WheelCount*(float)Math.sin(Coords.Angle);
    }
    private void UpdateRotation(){
        if(Encoders[0] > 0 && Encoders[2] > 0 ){
            Coords.Angle += GetAvaragePosition()*WheelCount*RobotOneDeg;
        }
        else{
            Coords.Angle -= GetAvaragePosition()*WheelCount*RobotOneDeg;
        }
    }

    private void ResestMotors(){
        Motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
