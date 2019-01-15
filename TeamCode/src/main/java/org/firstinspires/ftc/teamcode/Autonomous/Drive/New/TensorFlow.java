package org.firstinspires.ftc.teamcode.Autonomous.Drive.New;

import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;

/**
 * Created by Simon on 19/12/2018.
 */

public class TensorFlow extends MainTask{

    public TensorFlow(TensorFlowCubeDetection TensorFlow,int position, MainTask[] LeftTask, MainTask[] MiddleTask, MainTask[] RightTask){
        Left = LeftTask;
        Middle = MiddleTask;
        Right = RightTask;
        tensorFlow = TensorFlow;
        value = position;
        context = "CubeDetection";
        disiredTime = System.nanoTime()+((long)3*1000000000);
    }
}
