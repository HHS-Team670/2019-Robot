package frc.team670.robot.utils.sort;
import frc.team670.robot.utils.sort.Node;

public class TestNode implements Node{

    private int x, y;
    private Edge[] edges;

    public TestNode(int x, int y, Edge[] edges){
        this.x = x;
        this.y = y;
        this.edges = edges;
    }

    public Edge[] getEdges(){
        return edges;
    }

    public int getX(){
        return x;
    }

    public int getY(){
        return y;
    }

    public int getHeuristicDistance(Node target){
        return 0; //TODO figure this out
    }

    public int getHeuristicDistance(TestNode target){
        return (int)(Math.sqrt((x-target.getX())*(x-target.getX())+(y-target.getY())*(y-target.getY())));
    }
}