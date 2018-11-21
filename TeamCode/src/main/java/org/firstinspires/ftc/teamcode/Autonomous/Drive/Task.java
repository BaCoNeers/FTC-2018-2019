package org.firstinspires.ftc.teamcode.Autonomous.Drive;

/**
 * Created by Simon on 21/11/2018.
 */

public class Task {


    public float Power;
    public float Angle = 0;
    public float Forward = 0;


    public Task(float value, float Power, String Context){
        if(Context == "Forward"){
            this.Forward = value;
            this.Power = Power;
        }
        if(Context == "Turning"){
            this.Angle = value;
            this.Power = Power;
        }

    }


}
