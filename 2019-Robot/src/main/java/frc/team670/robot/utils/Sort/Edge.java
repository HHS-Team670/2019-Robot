package frc.team670.robot.utils.Sort;
import java.awt.geom.Point2D;

public interface Edge{
    /**
     * @return the "cost" of travelling between nodes (distance from 1 node to another)
     */
    public double getCost();
    /**
     * @return the source node (where this edge starts)
     */
    public Node getSource();
    /*
     * @return the destination node (where this edge ends)
     */        
    public Point2D.Double getDest();
}
