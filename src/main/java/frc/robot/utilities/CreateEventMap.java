// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.utilities.Constants.*;

public class CreateEventMap {

  HashMap<String, Command> eventMap = new HashMap<>();

  SwerveDrive m_swerveDrive;

    /**
     * Creates an Event Map for pathplanner autos
     * 
     * @param m_swerveDrive
     * 
     */
    public CreateEventMap(SwerveDrive m_swerveDrive, Arm m_arm, Claw m_claw) {
    this.m_swerveDrive = m_swerveDrive;
    
    eventMap.put("midCone", new InstantCommand(() -> m_arm.setPosition(MID_CONE_SETPOINT)));
  }


    /**
     * Returns an event map created in the CreateEventMap.java file
     * 
     * 
     */
  public HashMap<String, Command> createMap(){
    return eventMap;
  }
}
