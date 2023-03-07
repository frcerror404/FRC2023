// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWristSpeedForTime extends CommandBase {
  private final Wrist m_wrist;
  private boolean firstLoop = true;
  private double m_startTime = 0, m_delaySeconds = 0, m_wristSpeed = 0;


  /** Creates a new SetWristSpeedForTime. */
  public SetWristSpeedForTime(Wrist wrist, double wristSpeed, double delaySeconds) {

    m_wrist = wrist;
    m_delaySeconds = delaySeconds;
    m_wristSpeed = wristSpeed;

    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(firstLoop) {
      m_startTime = Timer.getFPGATimestamp();
      firstLoop = false;
    }
    m_wrist.setWristSpeed(m_wristSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - m_startTime >= m_delaySeconds;
  }
}
