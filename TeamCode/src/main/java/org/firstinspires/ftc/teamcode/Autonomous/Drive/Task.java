package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;

/**
 * Created by Simon on 21/11/2018.
 */

public class Task {


    public final float Power;
    public final float Value;
    public final String Context;

    public final long disiredTime;

    public TensorFlowCubeDetection tensorFlow;

    public int maxLoop = 1000;


    public Task(float value, float Power, String Context){
        this.Context = Context;
        this.Power = Math.abs(Power);
        this.Value = value;
        disiredTime = 0;
    }
    public Task(float sleep){
        this.Context = "sleep";
        this.Power = 0;
        this.Value = 0;
        disiredTime = (System.currentTimeMillis()/1000)+(long)sleep;
    }
    public Task(TensorFlowCubeDetection tensorFlow){
        this.Context = "CubeDetection";
        this.Power = 0;
        this.Value = 0;
        disiredTime = (System.currentTimeMillis()/1000) + 5;
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
