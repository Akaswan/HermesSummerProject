// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.utilities.Constants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class ToggleClaw extends InstantCommand {    
  private Claw m_claw;
 
  public ToggleClaw(Claw claw) {
    m_claw = claw;

    addRequirements(m_claw);
  }

  @Override
  public void initialize() {
    if (m_claw.clawClosed) {
      m_claw.setPosition(OPEN_CLAW_ROTATIONS);
      m_claw.setClawClosed(false);
    } else {
      m_claw.setPosition(CLAW_MIN_ROTATIONS);
      m_claw.setClawClosed(true);
    }
  }
}
