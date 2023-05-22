// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utilities.Constants.*;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CreatePath;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.RunTestMotor;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.APTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TestMotor;
import frc.robot.utilities.CreateEventMap;

public class RobotContainer {

  // Controllers \\
  public static XboxController m_driverController = new XboxController(0);

  // Subsystems \\
  public static final SwerveDrive m_swerveDrive = new SwerveDrive();
  public static final Arm m_arm = new Arm();
  public static final Claw m_claw = new Claw();
  public static final APTag m_apTag = new APTag();
  public static final TestMotor m_testMotor = new TestMotor();

  public static HashMap<String, Command> eventMap = new HashMap<>();
  CreateEventMap createMap = new CreateEventMap(m_swerveDrive, m_arm, m_claw);

  // Delcare Auto Plays \\
  private final Command play1, play2;

  // Sendable Chooser For Auto \\
  public static SendableChooser<Command> m_auto_chooser;

  public static ParallelCommandGroup[] lowGridCommands;

  public static ParallelCommandGroup[] midGridCommands;

  public RobotContainer() {

    eventMap = createMap.createMap();

    play1 = new WaitCommand(1);

    play2 = new CreatePath(null, m_swerveDrive, "Test", 1.0, 1.5, null);

    m_auto_chooser = new SendableChooser<>();

    m_auto_chooser.setDefaultOption("Do Nothing", play1);
    m_auto_chooser.addOption("Test", play2);

    SmartDashboard.putData(m_auto_chooser);

    lowGridCommands = new ParallelCommandGroup[] { 
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_0), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_1), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_2), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_3), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_4), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_5), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_6), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_7), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_8), new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT)))};

    midGridCommands = new ParallelCommandGroup[] {
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_0), new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_1), new InstantCommand(() -> m_arm.setPosition(MID_CUBE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_2), new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_3), new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_4), new InstantCommand(() -> m_arm.setPosition(MID_CUBE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_5), new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_6), new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_7), new InstantCommand(() -> m_arm.setPosition(MID_CUBE_SETPOINT))),
        new ParallelCommandGroup(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_8), new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT)))};

    m_swerveDrive.setDefaultCommand(
      new TeleopSwerve(
          m_swerveDrive, 
          () -> -m_driverController.getRawAxis(XboxController.Axis.kLeftY.value), 
          () -> -m_driverController.getRawAxis(XboxController.Axis.kLeftX.value), 
          () -> -m_driverController.getRawAxis(XboxController.Axis.kRightX.value), 
          () -> new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).getAsBoolean())
      );

    m_arm.setDefaultCommand(new ManualArmControl(ARM_MANUAL_SPEED, m_arm));

    configureBindings();
  }

  private void configureBindings() {
    // Driver Controls \\
    new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> m_swerveDrive.zeroGyro()));
    new JoystickButton(m_driverController, XboxController.Button.kBack.value).onTrue(new FollowPath(m_swerveDrive, m_swerveDrive.getPose(), GRID_1));

    // Co-Driver Controls \\
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT)));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> m_arm.setPosition(MID_CUBE_SETPOINT)));
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> m_arm.setPosition(LOW_GOAL_SETPOINT)));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new ToggleClaw(m_claw));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).onTrue(new RunTestMotor(0.5, m_testMotor));
  }

  public Command getAutonomousCommand() {
    return m_auto_chooser.getSelected();
  }
}
