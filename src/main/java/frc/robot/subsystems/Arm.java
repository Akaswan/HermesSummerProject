// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static double kDt = 0.02;
  public static double intendedPosition = 0.0;

  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;

  private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(1.75, 0.75);
  private final ProfiledPIDController m_controller =
    new ProfiledPIDController(1.3, 0.0, 0.7, m_constraints, kDt);


  public Arm() {
    m_armMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armEncoder = m_armMotor.getEncoder();

    m_armEncoder.setPositionConversionFactor(1.0 / 360.0 * 2.0 * Math.PI * 1.5); // Do some measurements to figure this out
  }

  @Override
  public void periodic() {
    m_armMotor.set(m_controller.calculate(m_armEncoder.getPosition(), intendedPosition));

    SmartDashboard.putNumber("Intended Position", intendedPosition);
  }

  public void setPosition(double setPoint) {
    intendedPosition = setPoint;
  }
}
