// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.RunTestMotor;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TestMotor;

import static frc.robot.Constants.*;

public class RobotContainer {

  public static XboxController m_driverController = new XboxController(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  public static final Arm m_arm = new Arm();
  public static final Claw m_claw = new Claw();
  public static final TestMotor m_testMotor = new TestMotor();
  public static CTREConfigs ctreConfigs = new CTREConfigs();

  public RobotContainer() {
    m_swerveDrive.setDefaultCommand(
      new TeleopSwerve(
          m_swerveDrive, 
          () -> -m_driverController.getRawAxis(translationAxis), 
          () -> -m_driverController.getRawAxis(strafeAxis), 
          () -> -m_driverController.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
      )
  );
    m_arm.setDefaultCommand(new ManualArmControl(ARM_MANUAL_SPEED, m_arm));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new SetArmPosition(MID_CONE_SETPOINT, m_arm));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new SetArmPosition(MID_CUBE_SETPOINT, m_arm));
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new SetArmPosition(LOW_GOAL_SETPOINT, m_arm));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new ToggleClaw(m_claw));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).onTrue(new RunTestMotor(0.5, m_testMotor));

    zeroGyro.onTrue(new InstantCommand(() -> m_swerveDrive.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
