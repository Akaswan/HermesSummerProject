package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    // Global Constants \\
    public static final double KDT = 0.02;
    
    public static final double STICK_DEADBAND = 0.06;

    // Arm Constants \\

    public static final int ARM_MOTOR_ID = 1;

    public static final double ARM_P = 1.0;
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.0;


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

    // Drivebase Constants \\

    public static final Pose2d GRID_0 = new Pose2d(Units.feetToMeters(2.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_1 = new Pose2d(Units.feetToMeters(4.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_2 = new Pose2d(Units.feetToMeters(6.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_3 = new Pose2d(Units.feetToMeters(8.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_4 = new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_5 = new Pose2d(Units.feetToMeters(12.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_6 = new Pose2d(Units.feetToMeters(14.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_7 = new Pose2d(Units.feetToMeters(16.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));
    public static final Pose2d GRID_8 = new Pose2d(Units.feetToMeters(18.0), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180));

    public static final COTSFalconSwerveConstants chosenModule =
    COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    public static final boolean CAN_CODER_INVERT = chosenModule.canCoderInvert;

    public static final boolean INVERT_GYRO = false;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
    * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47625;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47625;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0));

    public static final double MAX_SPEED = Units.feetToMeters(16.3);
    public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

    public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;
    public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
    public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

    public static final double DRIVE_KS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double DRIVE_KV = (1.51 / 12);
    public static final double DRIVE_KA = (0.27 / 12);

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.05; //TODO: This must be tuned to specific robot
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    public static final boolean DRIVE_MOTOR_INVERT = chosenModule.driveMotorInvert;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = chosenModule.angleKP;
    public static final double ANGLE_KI = chosenModule.angleKI;
    public static final double ANGLE_KD = chosenModule.angleKD;
    public static final double ANGLE_KF = chosenModule.angleKF;

    public static final boolean ANGLE_MOTOR_INVERT = chosenModule.angleMotorInvert;
    public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
    
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 12;
        public static final int angleMotorID = 11;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(210.1+90);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 18;
        public static final int angleMotorID = 17;
        public static final int canCoderID = 7;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(309.4+90);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 14;
        public static final int angleMotorID = 13;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(215.8+90);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 16;
        public static final int angleMotorID = 15;
        public static final int canCoderID = 5;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(213.2+90);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}
