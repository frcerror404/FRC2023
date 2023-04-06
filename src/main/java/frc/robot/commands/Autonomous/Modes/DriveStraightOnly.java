package frc.robot.commands.Autonomous.Modes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class DriveStraightOnly extends CommandBase {
  public final Drivebase drivebase;
  public final Gyro gyro;
  public final double timeout = 60.0;
  private double speedLeft;
  private double speedRight;
  private int phase = 0;
  PIDController chargingStation;

  // Constructor
  public DriveStraightOnly(Drivebase argDrivebase, Gyro gyro2) {
    drivebase = argDrivebase;
    gyro = gyro2;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedLeft = .4;
    speedRight = .4;

    drivebase.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (leftOffset) {
    //  speedLeft += 0.25;
    //} else if (rightOffset) {
    //  speedRight += 0.25;
    //}


    // Touch other side
    if(phase == 0) {
      if (drivebase.getRightDistance() < 12) {
        drivebase.manualControl(-speedLeft, -speedRight, true, false);
      } else {
        drivebase.manualControl(0, 0, false, false);
        phase = 1;
      }
    } else {
      drivebase.manualControl(0, 0, false, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drivebase.manualControl(0.0, 0.0, false, false);
    }

    drivebase.SetBrakeMode(false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
