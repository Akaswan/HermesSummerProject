// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utilities.Constants.*;

import java.util.HashMap;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.CreatePath;
// import frc.robot.commands.FollowPPPath;
// import frc.robot.commands.LimitSwitchPickUp;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualClawControl;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.APTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.CreateEventMap;

public class RobotContainer {

  // Controllers \\
  public static XboxController m_driverController = new XboxController(1);
  public static CommandPS4Controller m_ps5Controller = new CommandPS4Controller(0);

  // Subsystems \\
  public static final SwerveDrive m_swerveDrive = new SwerveDrive();
  public static final Arm m_arm = new Arm();
  public static final Claw m_claw = new Claw();
  public static final APTag m_apTag = new APTag();

  public static HashMap<String, Command> eventMap = new HashMap<>();
  CreateEventMap createMap = new CreateEventMap(m_swerveDrive, m_arm, m_claw);

  // Delcare Auto Plays \\
  private final Command play1, play2;

  // Sendable Chooser For Auto \\
  public static SendableChooser<Command> m_auto_chooser;

  // public static ParallelCommandGroup[] lowGridCommands;

  // public static ParallelCommandGroup[] midGridCommands;

  public RobotContainer() {

    eventMap = createMap.createMap();

    play1 = new WaitCommand(1);

    play2 = new CreatePath(null, m_swerveDrive, "Test", 1.0, 1.5, null);

    m_auto_chooser = new SendableChooser<>();

    m_auto_chooser.setDefaultOption("Do Nothing", play1);
    m_auto_chooser.addOption("Test", play2);

    SmartDashboard.putData(m_auto_chooser);

    m_swerveDrive.setDefaultCommand(
      new TeleopSwerve(
          m_swerveDrive, 
          () -> -m_ps5Controller.getRawAxis(PS4Controller.Axis.kLeftY.value), 
          () -> -m_ps5Controller.getRawAxis(PS4Controller.Axis.kLeftX.value), 
          () -> -m_ps5Controller.getRawAxis(PS4Controller.Axis.kRightX.value), 
          () -> m_ps5Controller.R1().getAsBoolean())
      );

    m_arm.setDefaultCommand(new ManualArmControl(-ARM_MANUAL_SPEED, m_arm));
    m_claw.setDefaultCommand(new ManualClawControl(-ARM_MANUAL_SPEED, m_claw));

    configureBindings();
  }

  private void configureBindings() {
    // Driver Controls \\
    // new JoystickButton(m_ps5Controller, PS4Controller.Button.).onTrue(new FollowPPPath(m_swerveDrive, m_swerveDrive.getPose(), new Pose2d(0, Units.feetToMeters(4), Rotation2d.fromDegrees(0))));
    // new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> m_swerveDrive.zeroGyro()));
    // new JoystickButton(m_driverController, XboxController.Button.kBack.value).onTrue(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_1));
    m_ps5Controller.triangle().onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_claw.setClawClosed(false)), new InstantCommand(() -> m_claw.setPosition(OPEN_CLAW_ROTATIONS))));
    m_ps5Controller.square().onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_claw.setClawMaxAmperage(450)), new InstantCommand(() -> m_claw.setPosition(CLAW_MIN_ROTATIONS))));
    m_ps5Controller.circle().onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_claw.setClawMaxAmperage(175)), new InstantCommand(() -> m_claw.setPosition(CLAW_MIN_ROTATIONS + 20))));
    m_ps5Controller.cross().onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_claw.setPosition(10))));

    m_ps5Controller.povUp().onTrue(new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT)));
    m_ps5Controller.povRight().onTrue(new InstantCommand(() -> m_arm.setPosition(PICK_UP)));
    m_ps5Controller.povDown().onTrue(new InstantCommand(() -> m_arm.setPosition(STOW_SETPOINT)));
    m_ps5Controller.povLeft().onTrue(new ToggleClaw(m_claw));
    

    // Co-Driver Controls \\
    // new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT)));
    // new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> m_arm.setPosition(MID_CUBE_SETPOINT)));
    // new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT)));
    // new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_arm.setPosition(STICK_DEADBAND)));
    // new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_claw.setPosition(CLAW_MAX_ROTATIONS)));
    // new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> m_claw.setPosition(CLAW_MIN_ROTATIONS)));
    // new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_claw.setPosition(10)));
    // new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new ToggleClaw(m_claw));
    // new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new LimitSwitchPickUp(m_claw));
  }

  public Command getAutonomousCommand() {
    return m_auto_chooser.getSelected();
  }
}
