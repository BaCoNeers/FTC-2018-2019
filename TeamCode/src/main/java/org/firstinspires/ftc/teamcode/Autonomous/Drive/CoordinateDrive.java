package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.Main.Main;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;

/**
 * Created by Simon on 27/09/2018.
 */

public class CoordinateDrive extends Configuration {

    private Coordinates Coords = new Coordinates(0,0,0);
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];
    private float RobotCirumfrance = 957.557f;
    private float RobotOneDeg = RobotCirumfrance/360;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/1440;

    public void SetCoordinate(float x,float y){

    }
    private void Rotate(float deg, float Power){
        int direction = 1;
        if(deg<0){
            direction = -1;
        }
        float DisiredRotation = deg/RobotOneDeg;
        float Distance = DisiredRotation/WheelCount;
        MotorPower[0] = Power * (direction);
        MotorPower[1] = Power * (direction*-1);
        MotorPower[2] = Power * (direction);
        MotorPower[3] = Power * (direction*-1);
        ResestMotors();
        while(GetAvaragePosition()<Math.abs(deg)){
            UpdateEncoders();
            UpdateMotor(true);
            UpdatePowers();
        }
        UpdateMotor(false);
        UpdateRotation();
        ResestMotors();
    }
    private void Move(float mm , float Power){
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

    private float GetAngle(float x,float y) {
        return (float)Math.atan2(y,x);
    }
    public float GetAvaragePosition(){ return (Math.abs(Encoders[0])+Math.abs(Encoders[1])+Math.abs(Encoders[2])+Math.abs(Encoders[3]))/4;}

    private void UpdateMotor(boolean On){
        if(On){
            FrontLeft.setPower(MotorPower[0]);
            FrontRight.setPower(MotorPower[1]);
            BackLeft.setPower( MotorPower[2]);
            BackRight.setPower( MotorPower[3]);
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
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
