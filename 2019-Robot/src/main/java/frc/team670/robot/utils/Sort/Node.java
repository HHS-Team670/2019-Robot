package frc.team670.robot.utils.sort;

import java.awt.geom.Point2D;

import frc.team670.robot.subsystems.Arm.LegalState;

/**
* Models a node on a graph. Needed for A* sort 
* https://www.geeksforgeeks.org/a-search-algorithm/ 
*/
public interface Node extends Comparable<Node>{  
    
    /**
     * @return a list of the edges that hit this node
     */
    public Edge[] getEdges();
    
    /**
     * @return coordinates of the node
     */
    public Point2D.Double getCoord();

     /**
     * @return the state of the node
     */
    public LegalState getState();

    /**
     * @param the desired F value to set the Node to for AStarSearch
     */
    public void setF(double f);

    
     /**
     * @return the F value for AStarSearch
     */
    public double getF();

   
      /**
     * @param g  The desired G value to set the Node to for AStarSearch
     */
    public void setG(double g);

     /**
     * @return the G value for AStarSearch
     */
    public double getG();


    /**
     * @param parent The desired Parent to set the Node to for AStarSearch
     */
    public void setParent(Node parent);

    /**
     * @return the Parent for AStarSearch
     */
    public Node getParent();

    /**
     * @param other Another node to compare this node to
     */
    public int compareTo(Node other);
}