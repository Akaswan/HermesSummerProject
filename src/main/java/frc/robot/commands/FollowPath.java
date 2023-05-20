// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utilities.CreateTrajectory;

import static frc.robot.utilities.Constants.*;

public class FollowPath extends CommandBase {

  private Trajectory m_trajectory;
  private SwerveDrive m_SwerveDrive;
  private CreateTrajectory m_createTrajectory;

  private double timeElapsed;
  
  public FollowPath(SwerveDrive swerveDrive, CreateTrajectory createTrajectory) {
    m_SwerveDrive = swerveDrive;
    m_createTrajectory = createTrajectory;

    timeElapsed = 0;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double mps = 0;

    for(SwerveModule mod : m_SwerveDrive.mSwerveMods){
      mps += mod.getState().speedMetersPerSecond;    
    }

    mps /= m_SwerveDrive.mSwerveMods.length;

    m_trajectory = m_createTrajectory.generateTrajectory(mps);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = m_trajectory.sample(timeElapsed);

    ChassisSpeeds adjustedSpeeds = m_SwerveDrive.trajController.calculate(
      m_SwerveDrive.getPose(), goal, goal.poseMeters.getRotation());

    SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

    m_SwerveDrive.setModuleStates(moduleStates);

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
