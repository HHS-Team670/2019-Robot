package frc.team670.robot.utils.Sort;

/**
* Models a node on a graph. Needed for A* sort 
* https://www.geeksforgeeks.org/a-search-algorithm/ 
*/
public interface Node {  
    
    /**
     * @return a list of the edges that hit this node
     */
    public Edge[] getEdges();
    
    /**
     * @return estimated distance to target node
     */
    public int getHeuristicDistance(Node target);

}