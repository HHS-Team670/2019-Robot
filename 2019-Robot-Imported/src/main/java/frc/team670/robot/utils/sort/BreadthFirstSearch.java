/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.sort;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

/**
 * Add your docs here.
 */
public class BreadthFirstSearch {

    private static List<Edge> result;

    private BreadthFirstSearch() {}

    public static List<Edge> search(Node start, Node destination) {

        // System.out.println("start: " + start.getClass().getName() + ", dest: " + destination.getClass().getName());
        if(start == destination){
            return new ArrayList<Edge>();
        }

        Queue<Edge> queue = new LinkedList<>();
        List<Edge> explored = new ArrayList<>();
        List<Edge> startEdges = Arrays.asList(start.getEdges());

        Map<Edge, Edge> parentNodes = new HashMap<Edge, Edge>();
        queue.addAll(startEdges);

        if(queue.size() < 1) {
            throw new IllegalArgumentException("Invalid input, check for island");
        }

        while(!queue.isEmpty()){
            Edge current = queue.remove();
            // System.out.println(current.getDest().getClass().getName());
            if(current.getDest() == destination) {
                List<Edge> shortestPath = new ArrayList<Edge>();
                shortestPath.add(current);
                Node node = current.getSource();
                // System.out.println("--------------Found Path--------------");
                while(node != start) {
                    // System.out.println(node.getClass().getName());
                    Edge upPath = parentNodes.get(current);
                    node = upPath.getSource();
                    current = upPath;
                    shortestPath.add(current);
                }
                Collections.reverse(shortestPath);
                return shortestPath;
            }
            else{
                List<Edge> edges = Arrays.asList(current.getDest().getEdges());
                for(Edge edge : edges) {
                    if(!explored.contains(edge)) {
                        parentNodes.put(edge, current);
                        queue.add(edge);
                        explored.add(edge);
                    }
                }
            }
            explored.add(current);
        }

        throw new IllegalArgumentException("Invalid input, check for island");

    }

}
