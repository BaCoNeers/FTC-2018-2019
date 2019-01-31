package org.firstinspires.ftc.teamcode.Autonomous.Drive.New;

import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;

import java.util.ArrayList;

/**
 * Created by Simon on 13/12/2018.
 */

public class MainTask {


    String context;
    float value;
    float power;

    long disiredTime;

    public TensorFlowCubeDetection tensorFlow;

    boolean LiftState;

    ArrayList<MainTask> Right;
    ArrayList<MainTask> Middle;
    ArrayList<MainTask> Left;

    static int maxLoop = 300;
    int currentLoop = 0;

    boolean CheckTask(){
        currentLoop++;
        if(currentLoop>=maxLoop){
            return false;
        }
        return true;
    }
}
