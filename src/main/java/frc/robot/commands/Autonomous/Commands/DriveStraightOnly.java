package frc.robot.commands.Autonomous.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class DriveStraightOnly extends CommandBase {
  public final Drivebase drivebase;
  public final Gyro gyro;
  public final double timeout = 60.0;
  private double startTime;
  private double speedLeft;
  private double speedRight;
  private double degrees;
  private double backward_degrees;
  private boolean m_tippedForward;
  private boolean m_tippedBackward;
  private boolean leftOffset;
  private boolean rightOffset;
  private int phase = 0;
  private int flipCount = 0;
  private boolean forward = true;
  private boolean prevFlip = true;
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
    startTime = Timer.getFPGATimestamp();
    degrees = -4.0;
    backward_degrees = 1;
    speedLeft = .4;
    speedRight = .4;

    drivebase.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tippedForward = gyro.getRoll() < degrees;
    m_tippedBackward = gyro.getRoll() > backward_degrees;
    leftOffset = gyro.getYaw() > 0;
    rightOffset = gyro.getYaw() < 0;
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
