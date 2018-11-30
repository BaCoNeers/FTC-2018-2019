package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;

/**
 * Created by Simon on 21/11/2018.
 */

public class Task {


    public final float Power;
    public final float Value;
    public final boolean LiftState;
    public final String Context;

    public final long disiredTime;

    public TensorFlowCubeDetection tensorFlow;

    public int maxLoop = 400;


    public Task(float value, float Power, String Context){
        this.Context = Context;
        this.Power = Math.abs(Power);
        this.Value = value;
        this.LiftState = false;
        disiredTime = 0;
    }
    public Task(boolean LiftState, float Power){
        this.LiftState = LiftState;
        this.Context = "Lift";
        this.Power = Power;
        this.Value = 0;
        this.disiredTime = 0;
    }
    public Task(float sleep,String Context){
        this.Context = Context;
        this.Power = 0;
        this.Value = 0;
        this.LiftState = false;
        disiredTime = System.nanoTime()+((long)sleep*1000000000);
    }
    public Task(TensorFlowCubeDetection tensorFlow){
        this.Context = "CubeDetection";
        this.Power = 0;
        this.Value = 0;
        this.LiftState = false;
        disiredTime = System.nanoTime()+((long)3*1000000000);
        this.tensorFlow = tensorFlow;
    }

    public boolean CheckTask(){
        maxLoop-=1;
        if(maxLoop<0){
            return false;
        }
        return true;
    }
}
