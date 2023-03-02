// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;

public class SetWristSpeed extends InstantCommand {
  private final double m_speed;
  private final Wrist m_wrist;

  public SetWristSpeed(Wrist wrist, double speed) {
    m_speed = speed;
    m_wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setWristSpeed(m_speed);
  }
}
