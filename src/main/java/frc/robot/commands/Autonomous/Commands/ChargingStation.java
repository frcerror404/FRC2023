package frc.robot.commands.Autonomous.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class ChargingStation extends CommandBase {
  public final Drivebase drivebase;
  public final Gyro gyro;
  public static WPI_TalonFX RightLead;
  public static WPI_TalonFX LeftLead;
  public final double timeout = 60.0;
  private double startTime;
  private double speedForward;
  private double speedBackward;
  private double degrees;
  private double backward_degrees;
  private boolean m_tippedForward = false;
  private boolean firstTime;
  private double LeftOffset;
  private double rightOffset;
  private double rightCount;
  private double leftCount;
  private boolean driveBackward;
  private boolean m_tippedBackward;
  PIDController chargingStation;

  // Constructor
  public ChargingStation(Drivebase argDrivebase, Gyro argGyro, WPI_TalonFX argRightLead, WPI_TalonFX argLeftLead) {
    drivebase = argDrivebase;
    gyro = argGyro;
    RightLead = argRightLead;
    LeftLead = argLeftLead;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    degrees = -4.0;
    backward_degrees = 1;
    speedForward = -.375;
    speedBackward = .375;
    firstTime = false;

    RightLead.setSelectedSensorPosition(0, 0, 0);
    LeftLead.setSelectedSensorPosition(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("DriveBackward", driveBackward);
    SmartDashboard.putNumber("Station RightEncoder", RightLead.getSelectedSensorPosition());
    SmartDashboard.putNumber("Station LeftEncoder", LeftLead.getSelectedSensorPosition());
    SmartDashboard.putNumber("RightOffset", rightOffset);
    SmartDashboard.putNumber("Right Count", rightCount);
    SmartDashboard.putNumber("Sensor Position 1.2", (RightLead.getSelectedSensorPosition() / 1.2));
    SmartDashboard.putBoolean("First Time", firstTime);
    SmartDashboard.putNumber("Right Encoder Rotations",
        drivebase.ConvertEncoder(RightLead.getSelectedSensorPosition()));
    m_tippedForward = gyro.getRoll() < degrees;
    m_tippedBackward = gyro.getRoll() > backward_degrees;
    if (6 > drivebase.ConvertEncoder(RightLead.getSelectedSensorPosition())) {
      drivebase.manualControl(speedForward, speedForward, false);
    } else {
      drivebase.manualControl(0.2, 0.2, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drivebase.manualControl(0.0, 0.0, false);
    }
    RightLead.setNeutralMode(NeutralMode.Coast);
    LeftLead.setNeutralMode(NeutralMode.Coast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTime = Timer.getFPGATimestamp();
    return Math.abs(currTime - startTime) > timeout;
  }
}
