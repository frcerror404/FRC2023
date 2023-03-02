// Key subsystems such as the main subsystem file and constants.
package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import Talon (Falcon 500 motor controllers have talons in them.) libraries.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  // Left Master and Save motor controller.
  private final WPI_TalonFX LeftLead = new WPI_TalonFX(Constants.klDT1);
  private final WPI_TalonFX LeftFollow = new WPI_TalonFX(Constants.klDT2);

  // Right Master and Slave motor controller.
  private final WPI_TalonFX RightLead = new WPI_TalonFX(Constants.krDT1);
  private final WPI_TalonFX RightFollow = new WPI_TalonFX(Constants.krDT2);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(LeftLead, LeftFollow);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(RightLead, RightFollow);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);
  private final Gyro m_gyro = new Gyro();
  private final DifferentialDriveOdometry m_Odometry;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(18.86));

  public Drivebase() {

    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

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

  public void zeroEncoders() {
    LeftLead.setSelectedSensorPosition(0);
    RightLead.setSelectedSensorPosition(0);
  }

  public double getLeftDistance() {
    return ConvertEncoder(getLeftLeadSensor());
  }

  public double getRightDistance() {
    return ConvertEncoder(getRightLeadSensor());
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LeftLead.getStatorCurrent(), RightLead.getStatorCurrent());
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
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
    //m_Odometry.update(m_gyro.get2dRotation(), getRightLeadSensor(), getLeftLeadSensor());

    // System.out.print("Left Front: ");
    // System.out.print(LeftLead.getMotorOutputPercent());
    // System.out.print(", Left Rear: ");
    // System.out.println(LeftFollow.getMotorOutputPercent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void manualControl(double rawAxis, double rawAxis2, boolean turbo, boolean slow) {

    if(!turbo && !slow) {
      rawAxis *= Constants.kDrivetrainSpeedMultiplier;
      rawAxis2 *= Constants.kDrivetrainSpeedMultiplier;
    }

    if(slow) {
      rawAxis *= Constants.kDrivetrainLowSpeedMultiplier;
      rawAxis2 *= Constants.kDrivetrainLowSpeedMultiplier;
    }

    if (Constants.isCurvatureDrive) {
      drivetrain.curvatureDrive(rawAxis, rawAxis2, turbo);
    } else {
      drivetrain.tankDrive(rawAxis, rawAxis2);
    }

  }

  // Reset Motor Controllers and set Limits/Ramp Rate
  private void setLeftLeadDefaults() {
    LeftLead.configFactoryDefault();
    LeftLead.setNeutralMode(NeutralMode.Coast);
    LeftLead.configOpenloopRamp(.25);
  }

  private void setleftFollowDefaults() {
    LeftFollow.configFactoryDefault();
    LeftFollow.setNeutralMode(NeutralMode.Coast);
    LeftFollow.configOpenloopRamp(.25);
    LeftFollow.follow(LeftLead);
  }

  private void setRightLeadDefaults() {
    RightLead.configFactoryDefault();
    RightLead.setNeutralMode(NeutralMode.Coast);
    RightLead.configOpenloopRamp(.25);
  }

  private void setrightFollowDefaults() {
    RightFollow.configFactoryDefault();
    RightFollow.setNeutralMode(NeutralMode.Coast);
    RightFollow.configOpenloopRamp(.25);
    RightFollow.follow(RightLead);

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

  public static Drivebase drivebase() {
    return null;
  }
}
