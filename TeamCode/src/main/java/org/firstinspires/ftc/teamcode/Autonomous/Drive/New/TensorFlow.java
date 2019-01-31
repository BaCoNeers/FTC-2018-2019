package org.firstinspires.ftc.teamcode.Autonomous.Drive.New;

import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;

import java.util.ArrayList;

/**
 * Created by Simon on 19/12/2018.
 */

public class TensorFlow extends MainTask{

    public TensorFlow(TensorFlowCubeDetection TensorFlow, ArrayList<MainTask> LeftTask, ArrayList<MainTask> MiddleTask, ArrayList<MainTask> RightTask){
        Left = LeftTask;
        Middle = MiddleTask;
        Right = RightTask;
        tensorFlow = TensorFlow;
        context = "CubeDetection";
        disiredTime = System.nanoTime()+((long)3*1000000000);
    }
}
