package org.firstinspires.ftc.teamcode.Autonomous.Drive;

/**
 * Created by Simon on 21/11/2018.
 */

public class Task {


    public float Power;
    public float Value = 0;
    public String Context;


    public Task(float value, float Power, String Context){
        this.Context = Context;
        this.Power = Power;
        this.Value = value;

    }


}
