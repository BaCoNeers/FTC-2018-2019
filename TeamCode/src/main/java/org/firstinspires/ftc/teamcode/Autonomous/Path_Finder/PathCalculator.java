package org.firstinspires.ftc.teamcode.Autonomous.Path_Finder;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.Coordinates;

/**
 * Created by Simon on 18/10/2018.
 */

public class PathCalculator implements Runnable {

    private Thread t;
    private String threadName;
    private Node[][] Nodes;
    private Coordinates Current;

    PathCalculator( String name, Node[][] _Nodes, Coordinates _Current) {
        threadName = name;
        Nodes = _Nodes;
        Current = _Current;
        System.out.println("Creating " +  threadName );
    }

    public void run() {
        System.out.println("Running " +  threadName );
        try {
            for(int i = 4; i > 0; i--) {
                System.out.println("Thread: " + threadName + ", " + i);
                // Let the thread sleep for a while.
                Thread.sleep(50);
            }
        } catch (InterruptedException e) {
            System.out.println("Thread " +  threadName + " interrupted.");
        }
        System.out.println("Thread " +  threadName + " exiting.");
    }

    public void start () {
        System.out.println("Starting " +  threadName );
        if (t == null) {
            t = new Thread (this, threadName);
            t.start ();
        }
    }
}
