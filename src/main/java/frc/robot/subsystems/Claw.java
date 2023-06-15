// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static frc.robot.utilities.Constants.*;

import java.util.ArrayList;
import java.util.Collections;

public class Claw extends SubsystemBase {
  private static double intendedPosition = 10;

  private final CANSparkMax m_clawMotor;
  private final RelativeEncoder m_clawEncoder;

  public final DigitalInput limitSwitch;

  private final CANCoder m_absEncoder;

  public boolean clawClosed = true;
  public double clawAmperageLimit = 1000;

  ArrayList<Double> amperageHistory = new ArrayList<Double>(Collections.nCopies(20, 0.0));
  double amperageHistorySum = 0.0;

  private final PIDController m_controller = 
    new PIDController(CLAW_P, CLAW_I, CLAW_D);


  public Claw() {
    m_clawMotor = new CANSparkMax(CLAW_MOTOR_ID, MotorType.kBrushless);
    m_clawMotor.restoreFactoryDefaults();
    m_clawMotor.setIdleMode(IdleMode.kBrake);
    m_clawEncoder = m_clawMotor.getEncoder();
    m_clawEncoder.setPosition(0.0);

    m_absEncoder = new CANCoder(CLAW_ENCODER_ID);
    m_absEncoder.configFactoryDefault();
    m_absEncoder.configAllSettings(Robot.ctreConfigs.clawCanCoderConfig);

    limitSwitch = new DigitalInput(9);

    m_controller.setTolerance(.5);

    // m_clawEncoder.setPositionConversionFactor(1.0 / 360.0 * 2.0 * Math.PI * 1.5); // Do some measurements to figure this out
  }

  @Override
  public void periodic() {
    amperageHistorySum = 0;
    m_clawMotor.set(m_controller.calculate(m_absEncoder.getAbsolutePosition(), intendedPosition));

    amperageHistory.remove(0);
    amperageHistory.add(m_clawMotor.getOutputCurrent());

    for (int i=0;i<amperageHistory.size();i++) {
      amperageHistorySum += amperageHistory.get(i);
    }

    if(amperageHistorySum >= clawAmperageLimit && clawClosed == false) {
      intendedPosition = m_absEncoder.getAbsolutePosition() - 2;
      clawClosed = true;
    }

    // getClosed();

    SmartDashboard.putNumber("Max Amperage", clawAmperageLimit);
    SmartDashboard.putNumber("Amperage history", amperageHistorySum);
    // SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    SmartDashboard.putNumber("Claw Abs Pos", m_absEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Game Piece", getGamePiece());
    SmartDashboard.putNumber("Claw Intended Position", intendedPosition);
    SmartDashboard.putNumber("Claw Relative Position", m_clawEncoder.getPosition());
    SmartDashboard.putBoolean("Claw Closed", clawClosed);
    SmartDashboard.putNumber("Claw Motor Amperage", m_clawMotor.getOutputCurrent());
  }

  public void setPosition(double setPoint) {
    intendedPosition = setPoint;
  }

  public int getGamePiece() {
    if (CUBE_MIN_SETPOINT < m_absEncoder.getAbsolutePosition() && m_absEncoder.getAbsolutePosition() < CUBE_MAX_SETPOINT) {
      return 1;
    } else if (CONE_MIN_SETPOINT < m_absEncoder.getAbsolutePosition() && m_absEncoder.getAbsolutePosition() < CONE_MAX_SETPOINT) {
      return 2;
    } else {
      return 0;
    }
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void setClawMaxAmperage(double maxAmperage) {
    clawAmperageLimit = maxAmperage;
  }

  public void setClawClosed(boolean closed) {
    clawClosed = closed;
  }

  public void manualControl(double multiplier) {
    double movement = 0;
    if (RobotContainer.m_ps5Controller.R1().getAsBoolean()) {
      movement = CLAW_MANUAL_SPEED * multiplier;
    } else if (RobotContainer.m_ps5Controller.L1().getAsBoolean()) {
      movement = -CLAW_MANUAL_SPEED * multiplier;

    }

    if (intendedPosition <= 5 && movement > 0){
      intendedPosition += movement;
    } else if (intendedPosition >= CLAW_MAX_ROTATIONS && movement < 0) {
      intendedPosition += movement;
    } else if (CLAW_MIN_ROTATIONS < intendedPosition && intendedPosition < CLAW_MAX_ROTATIONS) {
      intendedPosition += movement;
    }
  }
}
