// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
  private static double intendedPosition = 0.0;

  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;

  private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_VELOCITY);
  private final ProfiledPIDController m_controller =
    new ProfiledPIDController(ARM_P, ARM_I, ARM_D, m_constraints, KDT);


  public Arm() {
    m_armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armEncoder = m_armMotor.getEncoder();
    m_armEncoder.setPosition(0.0);

    // m_armEncoder.setPositionConversionFactor(1.0 / 360.0 * 2.0 * Math.PI * 1.5); // Do some measurements to figure this out
  }

  @Override
  public void periodic() {
    m_armMotor.set(m_controller.calculate(m_armEncoder.getPosition(), intendedPosition));

    SmartDashboard.putNumber("Arm Intended Position", intendedPosition);
    SmartDashboard.putNumber("Arm Relative Position", m_armEncoder.getPosition());
  }

  public void setPosition(double setPoint) {
    intendedPosition = setPoint;
  }

  public void manualControl(double multiplier) {
    double movement = RobotContainer.m_driverController.getRightY() * multiplier;

    if (RobotContainer.m_driverController.getRightY() > STICK_DEADBAND || RobotContainer.m_driverController.getRightY() < -STICK_DEADBAND) {
      if (intendedPosition <= ARM_MIN_ROTATIONS && movement > 0){
        intendedPosition += movement;
      } else if (intendedPosition >= ARM_MAX_ROTATIONS && movement < 0) {
        intendedPosition += movement;
      } else if (ARM_MIN_ROTATIONS < intendedPosition && intendedPosition < ARM_MAX_ROTATIONS) {
        intendedPosition += movement;
      }
    } else {
      if (intendedPosition > 32) {
        intendedPosition = 32;
      } else if (intendedPosition < 0) {
        intendedPosition = 0;
      }
    }
  }
}
