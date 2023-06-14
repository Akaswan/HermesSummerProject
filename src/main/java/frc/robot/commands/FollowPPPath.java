// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.utilities.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowPPPath extends InstantCommand {

  private PathPlannerTrajectory m_trajectory;
  private SwerveDrive m_swerveDrive;
  private Pose2d m_start;
  private Pose2d m_end;
  
  public FollowPPPath(SwerveDrive swerveDrive, Pose2d start, Pose2d end) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_start = start;
    m_end = end;

    addRequirements(m_swerveDrive);

    m_trajectory = m_swerveDrive.generateTrajectory(m_start, m_end, m_swerveDrive.getAngleToPose(start, end), m_swerveDrive.getAngleToPose(end, start));

    // SmartDashboard.putNumber("heading1", m_swerveDrive.getAngleToPose(start, end).getDegrees());
    // SmartDashboard.putNumber("heading2", m_swerveDrive.getAngleToPose(end, start).getDegrees());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new PPSwerveControllerCommand(
      m_trajectory, 
      m_swerveDrive::getPose, // Pose supplier
      SWERVE_KINEMATICS, // SwerveDriveKinematics
      new PIDController(2.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
      new PIDController(3.2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      m_swerveDrive::setModuleStates, // Module states consumer
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_swerveDrive // Requires this drive subsystem
    ).schedule();
  }
}
