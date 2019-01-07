package frc.team670.robot.constants.controllers;

import edu.wpi.first.wpilibj.Joystick;

public class MustangController extends Joystick {

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
        public static final int leftStickButton = 11;
        public static final int rightStickButton = 12;

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

    public boolean getAButton() {
        return super.getRawButton(XboxButtons.a);
    }

}