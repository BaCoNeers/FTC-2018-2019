package org.firstinspires.ftc.teamcode.Autonomous.Drive;

/**
 * Created by Simon on 21/11/2018.
 */

public class Task {


    public final float Power;
    public final float Value;
    public final String Context;

    private int maxLoop = 20000;


    public Task(float value, float Power, String Context){
        this.Context = Context;
        this.Power = Power;
        this.Value = value;

    }

    public boolean CheckTask(){
        maxLoop-=1;
        if(maxLoop<0){
            return false;
        }
        return true;
    }


}
