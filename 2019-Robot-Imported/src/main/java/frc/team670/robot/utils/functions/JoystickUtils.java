package frc.team670.robot.utils.functions;

/**
 * Class to hold functions which modify Joystick inputs to smooth them, moodify them, etc.
 * @author shaylandias
 */
public class JoystickUtils{

  /**
   * 
   * Runs a calculation to smooth out a joystick input in range [-1, 1] (currently squares it)
   * 
   * @param joystickVal The joystick raw input
   * @return Smoothed out joystick input
   */
  public static double smoothInput(double joystickVal){
    // if(joystickVal > 0)
    //   return joystickVal * joystickVal * joystickVal;
    // else
      return joystickVal * joystickVal * joystickVal;
  }

}