/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.sort;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.List;

import org.junit.Test;

import frc.team670.robot.utils.sort.AStarSearch;
import frc.team670.robot.utils.sort.Edge;

/**
 * Add your docs here.
 */
public class BreadthFirstSearchTest {

    @Test
    public void testBreadthFirstSearch() {
        TestEdge e0 = new TestEdge(), e1 = new TestEdge(), e2 = new TestEdge(), e3 = new TestEdge(), e4 = new TestEdge(), e5 = new TestEdge();
        TestNode t0 = new TestNode(0, 0, new Edge[]{e0, e1});
        TestNode t1 = new TestNode(1, 1, new Edge[]{e3, e5});
        TestNode t2 = new TestNode(1, 4, new Edge[]{e2});
        TestNode t3=new TestNode(4, 3, new Edge[]{e3, e4});
        TestNode t4=new TestNode(6, 2, new Edge[]{} ); //island
        TestNode t5= new TestNode(6, 4, new Edge[]{});
        e0.setNodes(t0, t1);
        e1.setNodes(t0, t2);
        e2.setNodes(t2, t5);
        e3.setNodes(t1, t3);
        e4.setNodes(t3, t5);
        e5.setNodes(t1, t0);
    
        List<Edge> result = BreadthFirstSearch.search(t0, t1);
        assertArrayEquals(new Edge[]{e0}, result.toArray());
        result = AStarSearch.search(t1, t0);
        assertArrayEquals(new Edge[]{e5}, result.toArray());
    
    }

}
