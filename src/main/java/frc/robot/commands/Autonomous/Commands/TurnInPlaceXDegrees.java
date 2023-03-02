// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class TurnInPlaceXDegrees extends CommandBase {
  private final Drivebase m_drivebase;
  private double m_speed;
  private final double m_degrees;
  private final Gyro _gyro;

  private double goal = 0.0;
  PIDController turnController;

  static final double kP = 0.012;
  static final double kI = 0.003;
  static final double kD = 0.0005;
  static final double max_output = 0.7;
  static final double kToleranceDegrees = 3.0f;
  boolean enabled = true;

  /** Creates a new TurnInPlaceXDegrees. */
  public TurnInPlaceXDegrees(Drivebase drivebase, Gyro gyro, double degrees, double speed) {
    m_drivebase = drivebase;
    m_degrees = degrees * .85;
    m_speed = speed;
    _gyro = gyro;

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goal = _gyro.getYaw() + m_degrees;

    if (m_degrees < 0) {
      m_speed = -m_speed;
    }

    turnController = new PIDController(kP, kI, kD);
    turnController.setSetpoint(goal);
    turnController.setIntegratorRange(0, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(goal - _gyro.getYaw());

    if (!enabled) {
      return;
    }

    double speed = turnController.calculate(_gyro.getYaw());
    speed = clamp(speed);
    m_drivebase.manualControl(-speed, speed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.manualControl(0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(_gyro.getYaw() - goal);
    return Math.abs(_gyro.getYaw() - goal) < kToleranceDegrees;
  }

  private double clamp(double input) {
    if (input > max_output) {
      return max_output;
    }
    if (input < -max_output) {
      return -max_output;
    }

    return input;

  }
}
