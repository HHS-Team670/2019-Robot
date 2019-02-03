package frc.team670.robot.utils.sort;

import frc.team670.robot.utils.Sort.Edge;
import frc.team670.robot.utils.Sort.Node;

public class TestEdge implements Edge{

    private TestNode start, dest;

    public TestEdge(){
        this.start = new TestNode(0, 0, new Edge[]{});
        this.dest = new TestNode(0, 0, new Edge[]{});
    }

    public TestEdge(TestNode start, TestNode dest){
        this.start = start;
        this.dest = dest;
    }

    public int getCost(){
        return (int)(Math.sqrt((dest.getX()-start.getX())*(dest.getX()-start.getX())+(dest.getY()-start.getY())*(dest.getY()-start.getY())));
    }
    public Node getSource(){
        return start;
    }
    public Node getDest(){
        return dest;
    }

    public String toString(){
        return ""+this.start.toString() + ","+this.dest.toString();
    }

    public void setNodes(TestNode start, TestNode dest){
        this.start = start;
        this.dest = dest;
    }


}