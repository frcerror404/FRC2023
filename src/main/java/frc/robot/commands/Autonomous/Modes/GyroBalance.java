// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Modes;

import java.sql.Time;

import org.ejml.dense.block.MatrixOps_MT_DDRB;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

@SuppressWarnings("unused")
public class GyroBalance extends CommandBase {
  private final Drivebase m_drivebase;
  private final Gyro m_gyro;

  private double baseSpeed = .65, slowSpeed = .32;
  private double toleranceDegrees = 2;
  private boolean onRamp = false;
  private double maxYaw = 0;

  private int sequence = 0;

  private double flattenThreshold  = 8.0;

  private double m_startTime = 0;
  private double m_timeout = 10.0;
  
  /** Creates a new GyroBalance. */
  public GyroBalance(Drivebase drivebase, Gyro gyro) {
    m_drivebase = drivebase;
    m_gyro = gyro;

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onRamp = false;
    sequence = 0;
    m_drivebase.SetBrakeMode(true);
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sequence == 0) { // off ramp, drive until yaw is positive
      m_drivebase.rawControl(baseSpeed, baseSpeed);

      if(getAngle() > 10) {
        sequence = 1;
      }
    }

    if(sequence == 1) { // drive until the robot starts tipping the other direction
      m_drivebase.rawControl(baseSpeed, baseSpeed);
      maxYaw = getAngle() > maxYaw ? getAngle() : maxYaw;

      if(maxYaw - getAngle() > flattenThreshold) {
        sequence = 2;
      }
    }

    if(sequence == 2) {

      if(getAngle() > 4) {
        m_drivebase.rawControl(slowSpeed, slowSpeed);
      } else if(getAngle() < -4) {
        m_drivebase.rawControl(-slowSpeed, -slowSpeed);
      } else {
        m_drivebase.rawControl(0, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private boolean isTimedOut() {
    return Timer.getFPGATimestamp() - m_startTime > m_timeout;
  }

  private boolean isFlat() {
    return Math.abs(m_gyro.getPitch()) < toleranceDegrees && onRamp;
  }

  private double getAngle() {
    return m_gyro.getRoll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFlat() || isTimedOut();
  }
}
