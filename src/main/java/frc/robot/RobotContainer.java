// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetArmPosition;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

public class RobotContainer {

  public static CommandXboxController m_driverController = new CommandXboxController(0);

  public static final Arm m_arm = new Arm();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.x().whileTrue(new SetArmPosition(MID_CONE_SETPOINT, m_arm));

    m_driverController.a().whileTrue(new SetArmPosition(MID_CUBE_SETPOINT, m_arm));

    m_driverController.b().whileTrue(new SetArmPosition(LOW_GOAL_SETPOINT, m_arm));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
