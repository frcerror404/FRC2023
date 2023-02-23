// Key subsystems such as the main subsystem file and constants.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import Talon (Falcon 500 motor controllers have talons in them.) libraries.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class old_Drivetrain extends SubsystemBase {
  // Left Master and Save motor controller.
  private final WPI_TalonFX LeftLead = new WPI_TalonFX(Constants.klDT1);
  private final WPI_TalonFX LeftFollow = new WPI_TalonFX(Constants.klDT2);

  // Right Master and Slave motor controller.
  private final WPI_TalonFX RightLead = new WPI_TalonFX(Constants.krDT1);
  private final WPI_TalonFX RightFollow = new WPI_TalonFX(Constants.krDT2);

  private final DifferentialDrive drivetrain = new DifferentialDrive(LeftLead, RightLead);
  private final Gyro m_gyro = new Gyro();
  private final DifferentialDriveOdometry m_Odometry;

  public old_Drivetrain() {
    setRightLeadDefaults();
    setLeftLeadDefaults();
    setrightFollowDefaults();
    setleftFollowDefaults();

    m_Odometry = new DifferentialDriveOdometry(m_gyro.get2dRotation(), getLeftLeadSensor(), getRightLeadSensor());
  }

  public void smartDashboardDrivetrainEncoders() {
    SmartDashboard.putNumber("Drivetrain RightLead", RightLead.getSelectedSensorPosition());
    SmartDashboard.putNumber("Drivetrain LeftLead", LeftLead.getSelectedSensorPosition());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    RightLead.setVoltage(rightVolts);
    LeftLead.setVoltage(leftVolts);
    drivetrain.feed();
  }

  public double getLeftLeadSensor() {
    return LeftLead.getSelectedSensorPosition();
  }

  public double getRightLeadSensor() {
    return RightLead.getSelectedSensorPosition();
  }

  public WPI_TalonFX getLeftLead() {
    return LeftLead;
  }

  public WPI_TalonFX getRightLead() {
    return RightLead;
  }

  public void resetLeadEncoders() {
    LeftLead.setSelectedSensorPosition(0.0);
    RightLead.setSelectedSensorPosition(0.0);
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LeftLead.getStatorCurrent(), RightLead.getStatorCurrent());
  }

  public void resetOdometry(Pose2d pose) {
    RightLead.setSelectedSensorPosition(0.0);
    LeftLead.setSelectedSensorPosition(0.0);

    m_Odometry.resetPosition(m_gyro.get2dRotation(), getLeftLeadSensor(), getRightLeadSensor(), pose);
  }

  @Override
  public void periodic() {
    drivetrain.feedWatchdog();

    // try {
    // SmartDashboard.putNumber("Drivetrain Temp (FL):",
    // LeftMaster.getTemperature());
    // SmartDashboard.putNumber("Drivetrain Temp (FR):",
    // LeftSlave.getTemperature());
    // SmartDashboard.putNumber("Drivetrain Temp (BL):",
    // RightMaster.getTemperature());
    // SmartDashboard.putNumber("Drivetrain Temp (BR):",
    // RightSlave.getTemperature());
    // } catch(Exception e) {
    // System.out.println("Temperature Stale");
    // }
    m_Odometry.update(m_gyro.get2dRotation(), getRightLeadSensor(), getLeftLeadSensor());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void manualControl(double rawAxis, double rawAxis2, boolean button) {

    if (Constants.isCurvatureDrive) {
      drivetrain.curvatureDrive(rawAxis, rawAxis2, button);
    } else {
      drivetrain.tankDrive(rawAxis, rawAxis2);
      // System.out.println(String.format("Drivetrain Speed: %f %f", -rawAxis,
      // -rawAxis2));
    }

  }

  // Reset Motor Controllers and set Limits/Ramp Rate
  private void setLeftLeadDefaults() {
    LeftLead.configFactoryDefault();
    LeftLead.setNeutralMode(NeutralMode.Coast);
    LeftLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
    LeftLead.configOpenloopRamp(.25);
    LeftLead.setInverted(InvertType.InvertMotorOutput);
  }

  private void setleftFollowDefaults() {
    LeftFollow.configFactoryDefault();
    LeftFollow.setNeutralMode(NeutralMode.Coast);
    LeftFollow.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
    LeftFollow.configOpenloopRamp(.25);
    LeftFollow.follow(LeftLead);
    // LeftSlave.setInverted(true);
  }

  private void setRightLeadDefaults() {
    RightLead.configFactoryDefault();
    RightLead.setNeutralMode(NeutralMode.Coast);
    RightLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
    RightLead.configOpenloopRamp(.25);
  }

  private void setrightFollowDefaults() {
    RightFollow.configFactoryDefault();
    RightFollow.setNeutralMode(NeutralMode.Coast);
    RightFollow.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
    RightFollow.configOpenloopRamp(.25);
    RightFollow.follow(RightLead);
    RightFollow.setInverted(InvertType.FollowMaster);

  }

  public void SetBrakeMode(boolean on) {
    NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

    RightLead.setNeutralMode(mode);
    LeftLead.setNeutralMode(mode);
    RightFollow.setNeutralMode(mode);
    LeftFollow.setNeutralMode(mode);
  }

  public double ConvertEncoder(double value) {
    double rotationsPerMeter = (Constants.EncoderUnits / 1) * (Constants.GearboxUnits / 1) * Constants.feetToMeter;
    return value / rotationsPerMeter;
  }
}
