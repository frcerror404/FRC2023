package frc.robot.commands.Autonomous.Modes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class ChargingStation extends CommandBase {
  public final Drivebase drivebase;
  public final Gyro gyro;
  public final double timeout = 60.0;
  private double speedLeft;
  private double speedRight;
  private int phase = 0;
  private int flipCount = 0;
  private boolean forward = true;
  private boolean prevFlip = true;
  PIDController chargingStation;

  // Constructor
  public ChargingStation(Drivebase argDrivebase, Gyro gyro2) {
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
      if (12 > drivebase.getRightDistance()) {
        drivebase.manualControl(-speedLeft, -speedRight, true, false);
      } else {
        drivebase.manualControl(speedLeft, speedRight, false, false);
        phase = 1;
      }
    }


    // Balance
    if(phase == 1) {
      if(flipCount > 10) {
        drivebase.manualControl(0, 0, false, false);
        return;
      }
  
      if (correctDistance()) {
        drivebase.manualControl(-speedLeft, -speedRight, true, false);
      } else {
        drivebase.manualControl(speedLeft, speedRight, true, false);
      }
  
      if(forward != prevFlip) {
        flipCount++;
      }
  
      prevFlip = forward;
      forward = !correctDistance();
    }
    



  }

  private boolean correctDistance() {
    return 5.75 > drivebase.getRightDistance();
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
