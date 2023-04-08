// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

@SuppressWarnings("unused")
public class QuickTurnXDegrees extends CommandBase {
  private final Drivebase m_drivebase;
  private final Gyro m_gyro;
  private boolean m_clockwise;
  private final double m_targetDegrees;

  private double m_rightEncoder_start = 0, m_leftEncoder_start = 0, m_gyroRotation_start = 0;
  private double m_yawGoal = 0;

  private boolean m_isAbsolute = false;


  private double k180DegreeEncoderDistance = 1000;

  private double kP = .0045, kI = 0.0027, kD = 0.00005;
  private PIDController m_rotationPID = new PIDController(kP, kI, kD);

  private double kToleranceDegrees = 2, kToleranceDegPerSec = 2 ;

  private double kBaseSpeed = 0.0;
  private double m_startTime, m_timeout = 2;

  
  public QuickTurnXDegrees(Drivebase drivebase, Gyro gyro, double degrees, boolean clockwise, double timeout, boolean absolute) {
    m_isAbsolute = absolute;
    m_drivebase = drivebase;
    m_gyro = gyro;
    m_clockwise = clockwise;
    m_targetDegrees = degrees;
    m_timeout = timeout;

    SmartDashboard.putNumber("Rotation_kP", kP);
    SmartDashboard.putNumber("Rotation_kI", kI);
    SmartDashboard.putNumber("Rotation_kD", kD);

    addRequirements(drivebase);
  }


  /** Creates a new Quick180. */
  public QuickTurnXDegrees(Drivebase drivebase, Gyro gyro, double degrees, boolean clockwise, double timeout) {
    m_drivebase = drivebase;
    m_gyro = gyro;
    m_clockwise = clockwise;
    m_targetDegrees = degrees;
    m_timeout = timeout;

    SmartDashboard.putNumber("Rotation_kP", kP);
    SmartDashboard.putNumber("Rotation_kI", kI);
    SmartDashboard.putNumber("Rotation_kD", kD);

    addRequirements(drivebase);
  }


  @Override
  public void initialize() {
    m_rightEncoder_start = m_drivebase.getRightDistance();
    m_leftEncoder_start = m_drivebase.getLeftDistance();
    m_gyroRotation_start = m_gyro.getYaw();
    m_startTime = Timer.getFPGATimestamp();

    if(m_clockwise) {
      m_yawGoal = m_gyroRotation_start - m_targetDegrees;
    } else {
      m_yawGoal = m_gyroRotation_start + m_targetDegrees;
    }

    if(m_isAbsolute) {
      m_yawGoal = m_targetDegrees;
    }

    m_rotationPID.setIntegratorRange(-0.65, 0.65);
    m_rotationPID.setTolerance(kToleranceDegrees, kToleranceDegPerSec);
    m_rotationPID.reset();

    kP = SmartDashboard.getNumber("Rotation_kP", kP);
    kI = SmartDashboard.getNumber("Rotation_kI", kI);
    kD = SmartDashboard.getNumber("Rotation_kD", kD);
    m_rotationPID.setPID(kP, kI, kD);

    SmartDashboard.putNumber("Rotation_Setpoint", m_yawGoal);
  }

  @Override
  public void execute() {
    double pidOutput = m_rotationPID.calculate(m_gyro.getYaw(), m_yawGoal);
    System.out.println("PID Output: " + pidOutput + ", PID Error " + m_rotationPID.getPositionError());
    SmartDashboard.putNumber("Rotation_PIDOutput", pidOutput);

    if(!m_clockwise) {
      m_drivebase.manualControl(kBaseSpeed - pidOutput, -kBaseSpeed + pidOutput, true, false);
    } else {
      m_drivebase.manualControl(-kBaseSpeed + pidOutput, kBaseSpeed - pidOutput, true, false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.manualControl(0, 0, false, false);
  }

  private boolean isTimedOut() {
    return Timer.getFPGATimestamp() - m_startTime > m_timeout;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotationPID.atSetpoint() || isTimedOut();
  }
}
