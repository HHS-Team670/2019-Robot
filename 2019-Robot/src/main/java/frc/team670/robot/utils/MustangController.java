package frc.team670.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class MustangController extends Joystick {

   public enum DPadState {
        NEUTRAl, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT;
   }

    public static class XboxButtons {
        // Controller Buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RIGHT_BUMPER = 6;
        public static final int LEFT_BUMPER = 5;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int LEFT_JOYSTICK_BUTTON = 9;
        public static final int RIGHT_JOYSTICK_BUTTON = 10;

        // Controller Axes
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int LEFT_STICK_X = 0;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int LEFT_STICK_Y = 1;
        /**
         * Left = Positive, Right = Negative [-1, 1]
         */
        public static final int LEFT_TRIGGER_AXIS = 2;
        /**
         * Pressed = Positive [0, 1]
         */
        public static final int RIGHT_TRIGGER_AXIS = 3;
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int RIGHT_STICK_X = 4;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int RIGHT_STICK_Y = 5;
    }

    public MustangController(int port) {
        super(port);
    }

    // helps you get varoius axis and buttons on the XBox controller
    public double getLeftStickX() {
        return super.getRawAxis(XboxButtons.LEFT_STICK_X);
    }

    public double getLeftStickY() {
        return super.getRawAxis(XboxButtons.LEFT_STICK_Y);
    }

    public double getLeftTriggerAxis() {
        return super.getRawAxis(XboxButtons.LEFT_TRIGGER_AXIS);
    }

    public double getRightTriggerAxis() {
        return super.getRawAxis(XboxButtons.RIGHT_TRIGGER_AXIS);
    }

    public double getRightStickX() {
        return super.getRawAxis(XboxButtons.RIGHT_STICK_X);
    }

    public double getRightStickY() {
        return super.getRawAxis(XboxButtons.RIGHT_STICK_Y);
    }

    public boolean getAButton() {
        return super.getRawButton(XboxButtons.A);
    }

    public boolean getBButton() {
        return super.getRawButton(XboxButtons.B);
    }

    public boolean getXButton() {
        return super.getRawButton(XboxButtons.X);
    }

    public boolean getYButton() {
        return super.getRawButton(XboxButtons.Y);
    }

    public boolean getLeftBumper() {
        return super.getRawButton(XboxButtons.LEFT_BUMPER);
    }

    public boolean getRightBumper() {
        return super.getRawButton(XboxButtons.RIGHT_BUMPER);
    }

    public boolean getBackButton() {
        return super.getRawButton(XboxButtons.BACK);
    }

    public boolean getStartButton() {
        return super.getRawButton(XboxButtons.START);
    }

    public boolean getLeftJoystickButton() {
        return super.getRawButton(XboxButtons.LEFT_JOYSTICK_BUTTON);
    }

    public boolean getRightJoystickButton() {
        return super.getRawButton(XboxButtons.RIGHT_JOYSTICK_BUTTON);
    }

    public int getPOVValue() {
        return super.getPOV();
    }

    // gets angle of the DPad on the XBox controller pressed with increments of 45 degree angle. 
    // returns neutal or -1 when nothing is pressed
    public DPadState getDPadState() {
        
        int angle = super.getPOV();

        if(angle == 0) {
            return DPadState.UP;
        } 
        else if(angle == 45){
            return DPadState.UP_RIGHT;
        }
        else if(angle == 90) {
            return DPadState.RIGHT;
        }
        else if(angle == 135){
            return DPadState.DOWN_RIGHT;
        }
        else if(angle == 180) {
            return DPadState.DOWN;
        }
        else if(angle == 225){
            return DPadState.DOWN_LEFT;
        }
        else if(angle == 270) {
            return DPadState.LEFT;
        }
        else if(angle == 315){
            return DPadState.UP_LEFT;
        } 
        else {
            return DPadState.NEUTRAl;
        }
        
       }

}