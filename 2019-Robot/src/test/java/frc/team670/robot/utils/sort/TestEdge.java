package frc.team670.robot.utils.sort;
import frc.team670.robot.utils.sort.Edge;

public class TestEdge implements Edge{

    private TestNode start, dest;

    public TestEdge(TestNode start, TestNode dest){
        this.start = start;
        this.dest = dest;
    }

    public int getCost(){
        return (int)(Math.sqrt((x-target.getX())*(x-target.getX())+(y-target.getY())*(y-target.getY())));
    }
    public Node getSource(){

    }
    public Node getDest(){

    }
}