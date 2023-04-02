// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorHeight extends CommandBase {
  private final Elevator m_elevator;
  private double m_height, m_timeout, m_startTime, m_tolerance = 1000;
  private boolean m_holdUntileTimeout = false;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator elevator, double height, double timeout, boolean holdUntilTimeout) {
    m_elevator = elevator;
    m_timeout = timeout;
    m_holdUntileTimeout = holdUntilTimeout;
    m_height = height;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_elevator.getElevatorPosition() - m_height > 0) {
      m_elevator.setElevatorSpeed(1);
    } else {
      m_elevator.setElevatorSpeed(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setElevatorSpeed(0);
  }

  private boolean isTimedOut() {
    return Timer.getFPGATimestamp() - m_startTime > m_timeout;
  }

  private boolean isAtCorrectHeight() {
    return Math.abs(m_elevator.getElevatorPosition() - m_height) < m_tolerance;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_holdUntileTimeout) {
      return isTimedOut();
    } else {
      return isTimedOut() || isAtCorrectHeight();
    }
  }
}
