// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveStraightXMeters extends CommandBase {
  public final Drivebase m_drivebase;
  public double m_distance, m_timeout, m_speed;

  public double m_startTime = 0, m_encoderStart = 0, m_tolerance = 0.25;

  /** Creates a new DriveXMeters. */
  public DriveStraightXMeters(Drivebase drivebase, double distance, double speed, double timeout) {
    m_drivebase = drivebase;
    m_distance = distance;
    m_timeout = timeout;
    m_speed = speed;

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_drivebase.SetBrakeMode(true);
    m_encoderStart = m_drivebase.getRightDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distanceToGo() > 0) {
      m_drivebase.rawControl(-m_speed, -m_speed);
    } else {
      m_drivebase.rawControl(m_speed, m_speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.rawControl(0, 0);
  }

  private boolean isTimedOut() {
    return Timer.getFPGATimestamp() - m_startTime > m_timeout;
  }

  private boolean isCorrectDistance() {
    return Math.abs(distanceToGo()) < m_tolerance;
  }

  private double distanceToGo() {
    return m_drivebase.getRightDistance() - m_encoderStart - m_distance;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isTimedOut() || isCorrectDistance();
  }
}
