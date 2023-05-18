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

import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.Collections;

public class Claw extends SubsystemBase {
  private static double intendedPosition = 0.0;

  private final CANSparkMax m_clawMotor;
  private final RelativeEncoder m_clawEncoder;

  ArrayList<Double> amperageHistory = new ArrayList<Double>(Collections.nCopies(20, 0.0));
  double amperageHistorySum = 0.0;

  private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(CLAW_MAX_VELOCITY, CLAW_MAX_VELOCITY);
  private final ProfiledPIDController m_controller =
    new ProfiledPIDController(CLAW_P, CLAW_I, CLAW_D, m_constraints, KDT);


  public Claw() {
    m_clawMotor = new CANSparkMax(CLAW_MOTOR_ID, MotorType.kBrushless);
    m_clawMotor.restoreFactoryDefaults();
    m_clawMotor.setIdleMode(IdleMode.kBrake);
    m_clawEncoder = m_clawMotor.getEncoder();
    m_clawEncoder.setPosition(0.0);

    // m_clawEncoder.setPositionConversionFactor(1.0 / 360.0 * 2.0 * Math.PI * 1.5); // Do some measurements to figure this out
  }

  @Override
  public void periodic() {
    m_clawMotor.set(m_controller.calculate(m_clawEncoder.getPosition(), intendedPosition));

    if (m_clawMotor.getOutputCurrent() > CLAW_MAX_AMPERAGE) {
      intendedPosition = m_clawEncoder.getPosition();
    }

    SmartDashboard.putNumber("Claw Intended Position", intendedPosition);
    SmartDashboard.putNumber("Claw Relative Position", m_clawEncoder.getPosition());
    SmartDashboard.putBoolean("Claw Closed", getClosed());
  }

  public void setPosition(double setPoint) {
    intendedPosition = setPoint;
  }

  public double getPosition() {
    return m_clawEncoder.getPosition();
  }

  public int getGamePiece() {
    if (CUBE_MIN_SETPOINT < m_clawEncoder.getPosition() && m_clawEncoder.getPosition() < CUBE_MAX_SETPOINT) {
      return 1;
    } else if (CONE_MIN_SETPOINT < m_clawEncoder.getPosition() && m_clawEncoder.getPosition() < CONE_MAX_SETPOINT) {
      return 2;
    } else {
      return 0;
    }
  }

  public boolean getClosed() {
    if (intendedPosition < CUBE_MAX_SETPOINT) {
      return true;
    } else {
      return false;
    }
  }

  private boolean detectGamePiece() {
    // keep the last 10 values of ampage in ampageHistory
    amperageHistory.remove(0);
    amperageHistory.add(m_clawMotor.getOutputCurrent());
    amperageHistorySum = 0.0;
    for (int i=0;i<amperageHistory.size();i++) {
      amperageHistorySum += amperageHistory.get(i);
    }
    // may neep to tune the max average ampage and/or number of samples
    // if amperage draw exceeds 19 amps for eachof the last 10 cycles assume over retract
    if ((amperageHistorySum / 20) > 19){
      System.out.println("amperage over!");
      return true;
    } else {
      return false;
    }
        
  }

  public void manualControl(double multiplier) {
    double movement = RobotContainer.m_driverController.getLeftY() * multiplier;

    if (RobotContainer.m_driverController.getRightY() > STICK_DEADBAND || RobotContainer.m_driverController.getRightY() < -STICK_DEADBAND) {
      if (intendedPosition <= CLAW_MIN_ROTATIONS && movement > 0){
        intendedPosition += movement;
      } else if (intendedPosition >= CLAW_MAX_ROTATIONS && movement < 0) {
        intendedPosition += movement;
      } else if (CLAW_MIN_ROTATIONS < intendedPosition && intendedPosition < CLAW_MAX_ROTATIONS) {
        intendedPosition += movement;
      }
    }
  }
}
