// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.utilities.Constants.*;

public class LimitSwitchPickUp extends CommandBase {
  /** Creates a new LimitSwitchPickUp. */

  private Claw m_claw;
  public LimitSwitchPickUp(Claw claw) {
    m_claw = claw;
    addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.setPosition(CLAW_MAX_ROTATIONS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_claw.getLimitSwitch()) {
      m_claw.setPosition(8);
    }
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
