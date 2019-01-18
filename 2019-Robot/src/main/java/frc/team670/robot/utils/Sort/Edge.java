package frc.team670.robot.utils.sort;

/**
 * Represents an edge that connects nodes. 
 */
public interface Edge{
    /**
     * @return the "cost" of travelling between nodes (distance from 1 node to another)
     */
    public double getCost(Node d1, Node d2);
    
    /**
     * @return the source node (where this edge starts)
     */
    public double getSource();

    /*
     * @return the destination node (where this edge ends)
     */        
    public Node getDest();
}
