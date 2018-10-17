package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Configuration.AutoConfig;

/**
 * Created by Simon on 27/09/2018.
 */

abstract public class CoordinateDrive extends AutoConfig {

    public Coordinates Coords = new Coordinates(0,0,0);
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];
    private float RobotCirumfrance = 957.557f;
    private float RobotOneDeg = RobotCirumfrance/360;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/1440;
    private float MarginOfError = 1f;

    private Coordinates Goal = null;
    private float Angle;
    private float Distence;


    //need work
    public boolean SetCoordinate(float x,float y, float Power){
        if(Goal == null){
            Goal = new Coordinates(x,y,0);
            SetAngle();
            SetMagnitude();
        }
        if(CheckRotation()){
            SetUpRotation(Power);
        }
        else{
            if(CheckMovement()) {
                SetUpMovement(Power);
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
        if (Angle < 0) {
            direction = -1;
        }
        MotorPower[0] = Power * (direction);
        MotorPower[1] = Power * (direction * -1);
        MotorPower[2] = Power * (direction);
        MotorPower[3] = Power * (direction * -1);
        ResestMotors();
    }
    private boolean CheckRotation(){

        if (GetLeftAvaragePosition()<Math.abs(Angle)){
            UpdateEncoders();
            UpdateMotor(true);
            UpdatePowers();
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
        int direction = 1;
        if(Distence<0){
            direction = -1;
        }
        MotorPower[0] = Power*(direction);
        MotorPower[1] = Power*(direction);
        MotorPower[2] = Power*(direction);
        MotorPower[3] = Power*(direction);
        ResestMotors();
    }
    private boolean CheckMovement(){
        if (GetAvaragePosition()<Math.abs(Distence)){
            UpdateEncoders();
            UpdateMotor(true);
            UpdatePowers();
            return true;
        }
        else {
            UpdateMotor(false);
            UpdatePosition();
            ResestMotors();
            return false;
        }
    }

    private void Rotate(float deg, float Power){
        int direction = 1;
        if (deg < 0) {
            direction = -1;
        }
        float DisiredRotation = deg / RobotOneDeg;
        float Distance = DisiredRotation / WheelCount;
        MotorPower[0] = Power * (direction);
        MotorPower[1] = Power * (direction * -1);
        MotorPower[2] = Power * (direction);
        MotorPower[3] = Power * (direction * -1);
        ResestMotors();

        while (GetLeftAvaragePosition()<Math.abs(deg)){
            UpdateEncoders();
            UpdateMotor(true);
            UpdatePowers();
        }
        UpdateMotor(false);
        UpdateRotation();
        ResestMotors();
    }
    private void Move(float mm , float Power){
        //need work
        float DisiredCount = mm/WheelCount;
        MotorPower[0] = Power;
        MotorPower[1] = Power;
        MotorPower[2] = Power;
        MotorPower[3] = Power;
        ResestMotors();
        while (GetAvaragePosition()<DisiredCount){
            UpdateEncoders();
            UpdateMotor(true);
            UpdatePowers();
        }
        UpdatePosition();
        UpdateMotor(false);
        ResestMotors();

    }

    private void SetAngle() {
        float DisiredRotation = (float)Math.atan2(Goal.y-Coords.x,Goal.x-Coords.y) / RobotOneDeg;
        Angle = DisiredRotation / WheelCount;
    }
    private void SetMagnitude(){
        Distence =  (float) (Math.sqrt(Math.pow(Goal.x-Coords.x,2)+Math.pow(Goal.x-Coords.y,2)))/WheelCount;
    };
    public float GetAvaragePosition(){ return (Math.abs(Encoders[0])+Math.abs(Encoders[1])+Math.abs(Encoders[2])+Math.abs(Encoders[3]))/4;}
    public float GetLeftAvaragePosition(){ return (Math.abs(Encoders[0]+Encoders[2])/2);}

    private void UpdateMotor(boolean On){
        if(On){
            FrontLeft.setPower(MotorPower[0]);
            FrontRight.setPower(MotorPower[1]);
            BackLeft.setPower( MotorPower[2]);
            BackRight.setPower( MotorPower[3]);
        }
        else {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }
    }

    private void UpdateEncoders(){
        Encoders[0] = FrontLeft.getCurrentPosition();
        Encoders[1] = FrontRight.getCurrentPosition();
        Encoders[2] = BackLeft.getCurrentPosition();
        Encoders[3] = BackRight.getCurrentPosition();
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
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




}
