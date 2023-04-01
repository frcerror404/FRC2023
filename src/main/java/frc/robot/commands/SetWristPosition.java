// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class SetWristPosition extends CommandBase {
  private final Wrist m_Wrist;

  /** Creates a new SetWristPosition. */
  public SetWristPosition(Wrist wrist) {
    m_Wrist = wrist;

    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void execute() {
    m_Wrist.setWristPosition(Constants.WristPlayerStation);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
