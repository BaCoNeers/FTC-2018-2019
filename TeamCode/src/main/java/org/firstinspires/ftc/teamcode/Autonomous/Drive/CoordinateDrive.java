package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import org.firstinspires.ftc.teamcode.Autonomous.Main.Main;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;

/**
 * Created by Simon on 27/09/2018.
 */

public class CoordinateDrive extends Configuration {

    private Coordinates Coords = new Coordinates(0,0,0);
    private float FrontLeftEncoder = FrontLeft.getCurrentPosition();
    private float FrontRightEncoder = FrontRight.getCurrentPosition();
    private float BackLeftEncoder = BackLeft.getCurrentPosition();
    private float BackRightEncoder = BackRight.getCurrentPosition();
    private float RobotCirumfrance = 957.557f;
    private float RobotOneDeg = RobotCirumfrance/360;
    private float WheelCirumfrance = 314.159f;
    private float WheelCount = WheelCirumfrance/1440;

    public void SetCoordinate(float x,float y){
    }
    private void Rotate(float deg){

    }
    private void Move(float mm , float Power){
        float DisiredCount = mm/WheelCount;
        float FrontLeftPower = Power;
        float FrontRightPower = Power;
        float BackLeftPower = Power;
        float BackRightPower = Power;
        while ((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4<DisiredCount){
            UpdateEncoders();
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);
            FrontLeftPower = FrontLeftPower*((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4);
            FrontRightPower = FrontRightPower*((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4);
            BackLeftPower = BackLeftPower*((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4);
            BackRightPower = BackRightPower*((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4);
            Coords.x = (float)(((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4)*Math.cos(Coords.Angle));
            Coords.y = (float)(((FrontLeftEncoder+FrontRightEncoder+BackLeftEncoder+BackRightEncoder)/4)*Math.sin(Coords.Angle));
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    private float GetAngle(float x,float y) {
        return (float)Math.atan2(y,x);
    }
    private void UpdateEncoders(){
        FrontLeftEncoder = FrontLeft.getCurrentPosition();
        FrontRightEncoder = FrontRight.getCurrentPosition();
        BackLeftEncoder = BackLeft.getCurrentPosition();
        BackRightEncoder = BackRight.getCurrentPosition();
    }


}
