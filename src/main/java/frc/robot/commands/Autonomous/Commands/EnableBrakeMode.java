// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivebase;

public class EnableBrakeMode extends InstantCommand {
  private final Drivebase m_drivebase;
  private final boolean m_enabled;

  public EnableBrakeMode(Drivebase drivebase, boolean enabled) {
    m_drivebase = drivebase;
    m_enabled = enabled;
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.SetBrakeMode(m_enabled);
  }
}
