package frc.robot.commands.Autonomous.Commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class ChargingStation1 extends CommandBase {
  public final Drivebase drivebase;
  public final Gyro gyro;
  public final double timeout = 60.0;
  private double startTime;
  private double speedForward;
  private double speedBackward;
  private double degrees;
  private double EncoderLeftOffset;
  private double EncoderRightOffset;
  private double currentLeftEncoderValue;
  private double currentRightEncoderValue;
  private boolean m_tippedForward = false;
  private boolean m_turned = false;
  private boolean firstTime;
  PIDController chargingStation;

  // Constructor
  public ChargingStation1(Drivebase argDrivebase, Gyro argGyro) {
    drivebase = argDrivebase;
    gyro = argGyro;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    firstTime = true;
    degrees = -4.0;
    speedForward = -.45;
    speedBackward = .45;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tippedForward = gyro.getRoll() < degrees;
    if (!m_tippedForward) {
      drivebase.manualControl(speedForward, speedForward, true, false);
    } else {
      drivebase.manualControl(speedBackward, speedBackward, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drivebase.manualControl(0.0, 0.0, false, false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTime = Timer.getFPGATimestamp();
    return Math.abs(currTime - startTime) > timeout;
  }
}
