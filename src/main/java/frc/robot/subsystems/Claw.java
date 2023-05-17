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

public class Claw extends SubsystemBase {
  private static double kDt = 0.02;
  private static double intendedPosition = 0.0;

  private static double minRotation = 0.0; // Figure this out later
  private static double maxRotation = 32.0; // Figure this out later

  private final CANSparkMax m_clawMotor;
  private final RelativeEncoder m_clawEncoder;

  ArrayList<Double> amperageHistory = new ArrayList<Double>(Collections.nCopies(20, 0.0));
  double amperageHistorySum = 0.0;

  private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(CLAW_MAX_VELOCITY, CLAW_MAX_VELOCITY);
  private final ProfiledPIDController m_controller =
    new ProfiledPIDController(CLAW_P, CLAW_I, CLAW_D, m_constraints, kDt);


  public Arm() {
    m_clawMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_clawMotor.restoreFactoryDefaults();
    m_clawMotor.setIdleMode(IdleMode.kBrake);
    m_clawEncoder = m_armMotor.getEncoder();
    m_clawEncoder.setPosition(0.0);

    m_clawEncoder.setPositionConversionFactor(1.0 / 360.0 * 2.0 * Math.PI * 1.5); // Do some measurements to figure this out
  }

  @Override
  public void periodic() {
    m_clawMotor.set(m_controller.calculate(m_armEncoder.getPosition(), intendedPosition));

    SmartDashboard.putNumber("Arm Intended Position", intendedPosition);
    SmartDashboard.putNumber("Arm Relative Position", m_armEncoder.getPosition());
  }

  public void setPosition(double setPoint) {
    intendedPosition = setPoint;
  }

  private boolean detectGamePiece(){
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
    double movement = RobotContainer.m_driverController.getLeftY() * multiplier

    if RobotContainer.m_driverController.getRightY() > ARM_DEADBAND or RobotContainer.m_driverController.getRightY() < -ARM_DEADBAND {
      if (intendedPosition <= minRotation && movement > 0){
        intendedPosition += movement;
      } else if (intendedPosition >= maxRotation && movement < 0) {
        intendedPosition += movement;
      } else if (minRotation < intendedPosition < maxRotation) {
        intendedPosition += movement;
      }
    }
  }
}
