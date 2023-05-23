package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static frc.robot.utilities.Constants.*;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowPath;

public class SwerveDrive extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private final AHRS gyro;

    public final Field2d m_field = new Field2d();

    public HolonomicDriveController trajController;

    private Pigeon2 m_pigeon = new Pigeon2(13, "rio"); // TODO pass in id and canbus CAN.pigeon);

    private double m_simYaw;

    public SwerveDrive() {
        gyro = new AHRS(SPI.Port.kMXP, (byte)50);
        zeroGyro();

        SmartDashboard.putData("Field", m_field);

        for (int i=0; i <= 8; i++) {
            SmartDashboard.putBoolean("Grid-" + i + " Low", false);
            SmartDashboard.putBoolean("Grid-" + i + " Mid", false);
        }


        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        m_pigeon.setYaw(0);

        swerveOdometry = new SwerveDriveOdometry(SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

        trajController = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0.25, .01, 
                new TrapezoidProfile.Constraints(6.28, 3.14)));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        if (RobotBase.isReal()) {
            return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - (-(gyro.getYaw()+180))) : Rotation2d.fromDegrees(-(gyro.getYaw()+180));
        } else {
            double[] ypr = new double[3];
            m_pigeon.getYawPitchRoll(ypr);
            return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
        }
        
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Rotation2d getAngleToPose(Pose2d start, Pose2d end) {
        double deltaX = end.getX() - start.getX();
        double deltaY = end.getY() - start.getY();
        double angleRad = Math.atan2(deltaY, deltaX);

        
        double angleDeg = Math.toDegrees(angleRad);

        
        angleDeg = (angleDeg + 360) % 360;

        return Rotation2d.fromDegrees(angleDeg);
    }

    public PathPlannerTrajectory generateTrajectory(Pose2d start, Pose2d end) {
        double mps = 0;

        for(SwerveModule mod : mSwerveMods){
          mps += mod.getState().speedMetersPerSecond;    
        }
        
        mps /= mSwerveMods.length;

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(start.getTranslation(), getAngleToPose(start, end), start.getRotation(), mps),
            new PathPoint(end.getTranslation(), getAngleToPose(start, end), end.getRotation()) 
        );

        return trajectory;
      }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        boolean[] gridLow = {
            SmartDashboard.getBoolean("Grid-0 Low", false),
            SmartDashboard.getBoolean("Grid-1 Low", false),
            SmartDashboard.getBoolean("Grid-2 Low", false),
            SmartDashboard.getBoolean("Grid-3 Low", false),
            SmartDashboard.getBoolean("Grid-4 Low", false),
            SmartDashboard.getBoolean("Grid-5 Low", false),
            SmartDashboard.getBoolean("Grid-6 Low", false),
            SmartDashboard.getBoolean("Grid-7 Low", false),
            SmartDashboard.getBoolean("Grid-8 Low", false),
            SmartDashboard.getBoolean("Grid-9 Low", false)
        };

        boolean[] gridMid = {
            SmartDashboard.getBoolean("Grid-0 Mid", false),
            SmartDashboard.getBoolean("Grid-1 Mid", false),
            SmartDashboard.getBoolean("Grid-2 Mid", false),
            SmartDashboard.getBoolean("Grid-3 Mid", false),
            SmartDashboard.getBoolean("Grid-4 Mid", false),
            SmartDashboard.getBoolean("Grid-5 Mid", false),
            SmartDashboard.getBoolean("Grid-6 Mid", false),
            SmartDashboard.getBoolean("Grid-7 Mid", false),
            SmartDashboard.getBoolean("Grid-8 Mid", false),
            SmartDashboard.getBoolean("Grid-9 Mid", false)
        };

        for (int i = 0; i < gridLow.length; i++) {
            if (gridLow[i]) {
                SmartDashboard.putBoolean("Grid-" + i + " Low", false);
                new ParallelCommandGroup(new FollowPath(RobotContainer.m_swerveDrive, getPose(), GRID_POSITIONS[i]), new InstantCommand(() -> RobotContainer.m_arm.setPosition(LOW_GOAL_SETPOINT))).schedule();
            }

            if (gridMid[i]) {
                SmartDashboard.putBoolean("Grid-" + i + " Mid", false);
                new ParallelCommandGroup(new FollowPath(RobotContainer.m_swerveDrive, getPose(), GRID_POSITIONS[i]), new InstantCommand(() -> RobotContainer.m_arm.setPosition(MID_CONE_SETPOINT))).schedule();
            }
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        m_field.setRobotPose(swerveOdometry.getPoseMeters());
        SmartDashboard.putNumber("X", Units.metersToFeet(swerveOdometry.getPoseMeters().getX()));
        SmartDashboard.putNumber("Y", Units.metersToFeet(swerveOdometry.getPoseMeters().getY()));
        SmartDashboard.putNumber("A", (swerveOdometry.getPoseMeters().getRotation().getDegrees()));
    }

    @Override
    public void simulationPeriodic() {
      ChassisSpeeds chassisSpeed = SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
      m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
  
      Unmanaged.feedEnable(20);
      m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
    }
}
