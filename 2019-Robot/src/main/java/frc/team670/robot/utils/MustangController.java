package frc.team670.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class MustangController extends Joystick {

   public enum DPadState {
        NEUTRAl, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT;
   }

    public static class XboxButtons {
        // Controller Buttons
        public static final int a = 1;
        public static final int b = 2;
        public static final int x = 3;
        public static final int y = 4;
        public static final int rightBumper = 6;
        public static final int leftBumper = 5;
        public static final int back = 7;
        public static final int start = 8;
        public static final int leftJoystickButton = 9;
        public static final int rightJoystickButton = 10;

        // Controller Axes
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int leftStickX = 0;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int leftStickY = 1;
        /**
         * Left = Positive, Right = Negative [-1, 1]
         */
        public static final int leftTriggerAxis = 2;
        /**
         * Pressed = Positive [0, 1]
         */
        public static final int rightTriggerAxis = 3;
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int rightStickX = 4;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int rightStickY = 5;
    }

    public MustangController(int port) {
        super(port);
    }

    public double getLeftStickX() {
        return super.getRawAxis(XboxButtons.leftStickX);
    }

    public double getLeftStickY() {
        return super.getRawAxis(XboxButtons.leftStickY);
    }

    public double getLeftTriggerAxis() {
        return super.getRawAxis(XboxButtons.leftTriggerAxis);
    }

    public double getRightTriggerAxis() {
        return super.getRawAxis(XboxButtons.rightTriggerAxis);
    }

    public double getRightStickX() {
        return super.getRawAxis(XboxButtons.rightStickX);
    }

    public double getRightStickY() {
        return super.getRawAxis(XboxButtons.rightStickY);
    }

    public boolean getAButton() {
        return super.getRawButton(XboxButtons.a);
    }

    public boolean getBButton() {
        return super.getRawButton(XboxButtons.b);
    }

    public boolean getXButton() {
        return super.getRawButton(XboxButtons.x);
    }

    public boolean getYButton() {
        return super.getRawButton(XboxButtons.y);
    }

    public boolean getLeftBumper() {
        return super.getRawButton(XboxButtons.leftBumper);
    }

    public boolean getRightBumper() {
        return super.getRawButton(XboxButtons.rightBumper);
    }

    public boolean getBackButton() {
        return super.getRawButton(XboxButtons.back);
    }

    public boolean getStartButton() {
        return super.getRawButton(XboxButtons.start);
    }

    public boolean getLeftJoystickButton() {
        return super.getRawButton(XboxButtons.leftJoystickButton);
    }

    public boolean getRightJoystickButton() {
        return super.getRawButton(XboxButtons.rightJoystickButton);
    }

    public int getPOVValue() {
        return super.getPOV();
    }

    public DPadState getDPadState() {
        
        int angle = super.getPOV();

        if(angle == 0) {
            return DPadState.UP;
        } 
        else if(angle == 180) {
            return DPadState.DOWN;
        }
        else if(angle == 270) {
            return DPadState.LEFT;
        }
        else if(angle == 90) {
            return DPadState.RIGHT;
        }
        else if(angle > 0 && angle < 90) {
            return DPadState.UP_RIGHT;
        }
        else if(angle >= 270 && angle != 0) {
            return DPadState.UP_LEFT;
        }
        else if(angle > 90 && angle < 180) {
            return DPadState.DOWN_RIGHT;
        }
        else if(angle > 180 && angle < 270) {
            return DPadState.DOWN_LEFT;
        }
        else {
            return DPadState.NEUTRAl;
        }
        
       }

}