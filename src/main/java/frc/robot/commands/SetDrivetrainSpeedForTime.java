/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class SetDrivetrainSpeedForTime extends CommandBase {
  private final Drivebase drivebase;
  private final double leftSpeed, rightSpeed;
  private final double durationS;

  private double startTime;

  /**
   * Creates a new SetDrivetrainSpeedForTime.
   */
  public SetDrivetrainSpeedForTime(double leftSpeed, double rightSpeed, double duration, Drivebase drivebase) {
    this.drivebase = drivebase;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    durationS = duration;

    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.manualControl(leftSpeed, rightSpeed, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.manualControl(0.0, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Timer.getFPGATimestamp() - startTime) >= durationS;
  }
}
