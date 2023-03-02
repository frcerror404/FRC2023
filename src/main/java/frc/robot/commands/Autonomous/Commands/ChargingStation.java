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
  private double speedLeft;
  private double speedRight;
  private double degrees;
  private double backward_degrees;
  private boolean m_tippedForward;
  private boolean m_tippedBackward;
  private boolean leftOffset;
  private boolean rightOffset;
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
    speedLeft = .375;
    speedRight = .375;

    RightLead.setSelectedSensorPosition(0, 0, 0);
    LeftLead.setSelectedSensorPosition(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tippedForward = gyro.getRoll() < degrees;
    m_tippedBackward = gyro.getRoll() > backward_degrees;
    leftOffset = gyro.getYaw() > 0;
    rightOffset = gyro.getYaw() < 0;
    if (leftOffset) {
      speedLeft += 0.25;
    } else if (rightOffset) {
      speedRight += 0.25;
    }

    if (6 > drivebase.ConvertEncoder(RightLead.getSelectedSensorPosition())) {
      drivebase.manualControl(speedLeft, speedRight, false, false);
    } else {
      drivebase.manualControl(-speedLeft, -speedRight, false, false);
    }

    speedLeft = 0.375;
    speedRight = 0.375;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drivebase.manualControl(0.0, 0.0, false, false);
    }
    RightLead.setNeutralMode(NeutralMode.Coast);
    LeftLead.setNeutralMode(NeutralMode.Coast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
