package org.firstinspires.ftc.teamcode.Autonomous.Path_Finder;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.Old.Coordinates;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Simon on 16/10/2018.
 */

public class Node {

    public Coordinates Coord;
    public boolean Walkable;
    public float Score;

    public List<Node> Children = new ArrayList<>();
    public Node Parent = null;

    public Node(Coordinates coords, Boolean _walkable, float _score){
        Coord = coords;
        Walkable = _walkable;
        Score = _score;
    }

    public void AddChild(Node Child){
        Children.add(Child);
    }
    public void RemoveChild(Node Child){
        for(int i=0;i<Children.size();i++){
            if(Children.get(i)==Child){
                Children.remove(i);
            }
        }
    }

    public void SetParent(Node _parent){
        if(Parent !=null){
            if(_parent.Score < Parent.Score){
                Parent = _parent;
            }
        }
        {
            Parent = _parent;
        }
    }

}
