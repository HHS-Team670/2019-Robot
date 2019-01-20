package frc.team670.robot.utils.sort;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Test;
import org.junit.Assert.*;

/**
 * Test implementation of A* search algorithm
 * @author ctchen, rghosh670
 */
public class AStarTest{

      @Test  
      public void AStarSearchTest(){
        TestEdge e0 = new TestEdge(), e1 = new TestEdge(), e2 = new TestEdge(), e3 = new TestEdge(), e4 = new TestEdge();
        TestNode t0 = new TestNode(0, 0, new Edge[]{e0, e1});
        TestNode t1 = new TestNode(1, 1, new Edge[]{e1, e3});
        TestNode t2 = new TestNode(1, 4, new Edge[]{e1, e2});
        TestNode t3=new TestNode(4, 3, new Edge[]{e3, e4});
        TestNode t4=new TestNode(6, 2, new Edge[]{} );
        TestNode t5= new TestNode(6, 4, new Edge[]{e2, e4});
        e0.setNodes(t0, t1);
        e1.setNodes(t0, t2);
        e2.setNodes(t2, t5);
        e3.setNodes(t1, t3);
        e4.setNodes(t3, t5);
        // Regular search test
        List<Edge> result = AStarSearch.search(t0, t5);
        assertArrayEquals(new Edge[]{e0, e3, e4}, result.toArray());
        // Case: starting point is island 
        try {
            assertEquals("Invalid input, check for island", AStarSearch.search(t4, t5));
            fail( "Should have thrown an exception" );
        } 
        catch (Exception e) {
            String expectedMessage = "Invalid input, check for island";
            assertEquals("Exception message must be correct", expectedMessage, e.getMessage() );
        }   
        // Case: start and destination are the same
        List<Edge> result2 = AStarSearch.search(t5, t5);
        assertArrayEquals(new Edge[]{}, result2.toArray());
        // Case: destination is island
        try {
            assertEquals("Invalid input, check for island", AStarSearch.search(t0, t4));
            fail( "Should have thrown an exception" );
        } 
        catch (Exception e) {
            String expectedMessage = "Invalid input, check for island";
            assertEquals( "Exception message must be correct", expectedMessage, e.getMessage() );
        }   
        // Regular search test
        List<Edge> result4 = AStarSearch.search(t0, t3);
        assertArrayEquals(new Edge[]{e0, e3}, result4.toArray());
    }
}

