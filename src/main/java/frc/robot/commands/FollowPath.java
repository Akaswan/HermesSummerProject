// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.utilities.Constants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class FollowPath extends CommandBase {

  private PathPlannerTrajectory m_trajectory;
  private SwerveDrive m_swerveDrive;
  private Pose2d m_start;
  private Pose2d m_end;

  private double timeElapsed;
  
  public FollowPath(SwerveDrive swerveDrive, Pose2d start, Pose2d end) {
    m_swerveDrive = swerveDrive;
    m_start = start;
    m_end = end;

    addRequirements(m_swerveDrive);
    timeElapsed = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeElapsed = 0;
    m_trajectory = m_swerveDrive.generateTrajectory(m_start, m_end, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

    m_swerveDrive.m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // PathPlannerState goal = m_trajectory.sample(timeElapsed);

    // ChassisSpeeds adjustedSpeeds = m_swerveDrive.trajController.calculate(
    //   m_swerveDrive.getPose(), goal);

    // SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

    // m_swerveDrive.setModuleStates(moduleStates);

    // SmartDashboard.putNumber("Trajectory X", goal.poseMeters.getX());
    // SmartDashboard.putNumber("Trajectory Y", goal.poseMeters.getY());
    // SmartDashboard.putNumber("Trajectory A", Units.radiansToDegrees(goal.curvatureRadPerMeter));
    // SmartDashboard.putNumber("Time Elapsed", timeElapsed);
    // SmartDashboard.putNumber("Total", m_trajectory.getTotalTimeSeconds());

    // timeElapsed += KDT;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeElapsed >= m_trajectory.getTotalTimeSeconds();
    // return timeElapsed >= 20;
  }
}
