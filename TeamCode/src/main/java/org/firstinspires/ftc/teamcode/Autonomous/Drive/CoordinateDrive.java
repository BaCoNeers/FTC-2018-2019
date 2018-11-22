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

    private float RobotCirumfrance = 957.557f;
    private float RobotOneDeg = RobotCirumfrance/360f;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/1440f;

    private Telemetry tel;

    public CoordinateDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, Telemetry tel){
        Motors[0] = FLM;
        Motors[1] = FRM;
        Motors[2] = BLM;
        Motors[3] = BRM;
        this.tel = tel;
    }


    public boolean Forward(float distance, float power){
        int direction = 1;
        if(direction<0){
            direction = -1;
        }
        if(GetAvaragePosition() < distance){
            Mot
        }
    }



    public float GetAvaragePosition(){
        return (Math.abs(Encoders[0])+Math.abs(Encoders[1])+Math.abs(Encoders[2])+Math.abs(Encoders[3]))/4f;
    }
    public float GetLeftAvaragePosition(){
        return (Math.abs(Encoders[0]+Encoders[2])*0.5f);
    }

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

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
