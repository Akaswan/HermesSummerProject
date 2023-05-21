// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.utilities.Constants.*;

public class FollowPath extends CommandBase {

  private Trajectory m_trajectory;
  private SwerveDrive m_swerveDrive;
  private Pose2d m_start;
  private Pose2d m_end;

  private double timeElapsed;
  
  public FollowPath(SwerveDrive swerveDrive, Pose2d start, Pose2d end) {
    m_swerveDrive = swerveDrive;
    m_start = start;
    m_end = end;

    timeElapsed = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_trajectory = m_swerveDrive.generateTrajectory(m_start, m_end);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = m_trajectory.sample(timeElapsed);

    ChassisSpeeds adjustedSpeeds = m_swerveDrive.trajController.calculate(
      m_swerveDrive.getPose(), goal, goal.poseMeters.getRotation());

    SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

    m_swerveDrive.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Trajectory X", adjustedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Y", adjustedSpeeds.vyMetersPerSecond);

    timeElapsed += KDT;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
