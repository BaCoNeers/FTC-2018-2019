package org.firstinspires.ftc.teamcode.Autonomous.Main;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.Coordinates;
import org.firstinspires.ftc.teamcode.Autonomous.Path_Finder.PathFinder;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;

/**
 * Created by Simon on 27/09/2018.
 */

@Autonomous(name="auto", group="Iterative Opmode")
public class Main extends OpMode {


    private PathFinder Path = new PathFinder();

    @Override
    public void init() {
        Path.AddTask(new Coordinates(10,10,0));
    }

    @Override
    public void loop() {
        Path.Update();
    }


}
