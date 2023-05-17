package frc.robot;

public class Constants {
    // Global Constants \\
    public static final double KDT = 0.02;

    // Arm Constants \\

    public static final int ARM_MOTOR_ID = 1;

    public static final double ARM_P = 1.0;
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.0;

    public static final double ARM_DEADBAND = 0.055;

    public static final double ARM_MAX_VELOCITY = 1.75;
    public static final double ARM_MAX_ACCELERATION = 0.75;

    public static final double ARM_MIN_ROTATIONS = 0.0;
    public static final double ARM_MAX_ROTATIONS = 32.0;

    public static final double ARM_MANUAL_SPEED = 1.0;

    public static final double MID_CONE_SETPOINT = 5.0;
    public static final double MID_CUBE_SETPOINT = 3.0;
    public static final double LOW_GOAL_SETPOINT = 1.0;
    public static final double STOW_SETPOINT = 0.0;

    // Claw Constants \\

    public static final int CLAW_MOTOR_ID = 2;

    public static final double CLAW_P = 1.0;
    public static final double CLAW_I = 0.0;
    public static final double CLAW_D = 0.0;

    public static final double CLAW_MAX_AMPERAGE = 20.0;

    public static final double CLAW_MIN_ROTATIONS = 0.0;
    public static final double CLAW_MAX_ROTATIONS = 32.0;

    public static final double CUBE_MIN_SETPOINT = 16.0;
    public static final double CUBE_MAX_SETPOINT = 18.0;

    public static final double CONE_MIN_SETPOINT = 4.0;
    public static final double CONE_MAX_SETPOINT = 6.0;

    public static final double CLAW_MAX_VELOCITY = 1.75;
    public static final double CLAW_MAX_ACCELERATION = 0.75;
}
