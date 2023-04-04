// Key subsystems such as the main subsystem file and constants.
package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import Talon (Falcon 500 motor controllers have talons in them.) libraries.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private Trajectory trajectorPath;

  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(18.86));

  public Drivebase() {

    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

    trajectorPath = null;

    setRightLeadDefaults();
    setLeftLeadDefaults();
    setrightFollowDefaults();
    setleftFollowDefaults();

    resetEncoders();
    m_Odometry = new DifferentialDriveOdometry(m_gyro.get2dRotation(), 0.0, 0.0);
  }

  public void smartDashboardDrivetrainEncoders() {
    SmartDashboard.putNumber("Drivetrain RightLead", convertEncoder(RightLead.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Drivetrain LeftLead", convertEncoder(LeftLead.getSelectedSensorPosition()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("Left Volts", -leftVolts);
    SmartDashboard.putNumber("Right Volts", -rightVolts);
    leftMotors.setVoltage(-leftVolts);
    rightMotors.setVoltage(-rightVolts);
    drivetrain.feed();
  }

  public double getHeading() {
    return m_gyro.get2dRotation().getDegrees();
  }

  public void zeroEncoders() {
    LeftLead.setSelectedSensorPosition(0);
    RightLead.setSelectedSensorPosition(0);
  }

  public double getLeftDistance() {
    return convertEncoder(getLeftLeadSensor());
  }

  public double getRightDistance() {
    return convertEncoder(getRightLeadSensor());
  }

  public double getLeftLeadSensor() {
    return LeftLead.getSelectedSensorPosition();
  }

  public double getRightLeadSensor() {
    return -1 * RightLead.getSelectedSensorPosition();
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

  public void resetEncoders() {
    LeftLead.setSelectedSensorPosition(0.0);
    LeftFollow.setSelectedSensorPosition(0.0);
    RightLead.setSelectedSensorPosition(0.0);
    RightFollow.setSelectedSensorPosition(0.0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(convertEncoder(LeftLead.getSelectedSensorVelocity()), convertEncoder(RightLead.getSelectedSensorVelocity()));
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_Odometry.resetPosition(m_gyro.get2dRotation(), 0.0, 0.0, pose);
  }

  public SequentialCommandGroup getPathCommand(String pathname) {
    Path jsonTrajectorPath = Filesystem.getDeployDirectory().toPath().resolve("pathweaver/output/first.wpilib.json");
    try {
      trajectorPath = TrajectoryUtil.fromPathweaverJson(jsonTrajectorPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to use file" + jsonTrajectorPath, exception.getStackTrace());
      System.out.println("Unable to use file");
    }

    this.resetOdometry(trajectorPath.getInitialPose());

    return new SequentialCommandGroup(
      new RamseteCommand(
        trajectorPath,
        this::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter, Constants.kaVoltSecondsSquarePErMeter),
        kinematics,
        this::getWheelSpeeds,
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        this::tankDriveVolts,
        this
      )
    );
  }

  @Override
  public void periodic() {
    drivetrain.feedWatchdog();

    m_Odometry.update(m_gyro.get2dRotation(), convertEncoder(LeftLead.getSelectedSensorPosition()), convertEncoder(LeftLead.getSelectedSensorPosition()));
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

    if(turbo) {
      SetBrakeMode(false);
    }
    
    if(slow) {
      SetBrakeMode(true);
    }

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

  public void rawControl(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(-leftSpeed, -rightSpeed);
  }

  // Reset Motor Controllers and set Limits/Ramp Rate
  private void setLeftLeadDefaults() {
    LeftLead.configFactoryDefault();
    LeftLead.setNeutralMode(NeutralMode.Coast);
    LeftLead.configOpenloopRamp(.25);
    LeftLead.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    LeftLead.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

  }

  private void setleftFollowDefaults() {
    LeftFollow.configFactoryDefault();
    LeftFollow.setNeutralMode(NeutralMode.Coast);
    LeftFollow.configOpenloopRamp(.25);
    LeftFollow.follow(LeftLead);
    LeftFollow.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    LeftFollow.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
  }

  private void setRightLeadDefaults() {
    RightLead.configFactoryDefault();
    RightLead.setNeutralMode(NeutralMode.Coast);
    RightLead.configOpenloopRamp(.25);
    RightLead.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    RightLead.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
  }

  private void setrightFollowDefaults() {
    RightFollow.configFactoryDefault();
    RightFollow.setNeutralMode(NeutralMode.Coast);
    RightFollow.configOpenloopRamp(.25);
    RightFollow.follow(RightLead);
    RightFollow.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    RightFollow.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
  }

  public void SetBrakeMode(boolean on) {
    NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

    RightLead.setNeutralMode(mode);
    LeftLead.setNeutralMode(mode);
    RightFollow.setNeutralMode(mode);
    LeftFollow.setNeutralMode(mode);
    Constants.breakMode = mode;
  }

  public double convertEncoder(double value) {
    double rotationsPerMeter = (Constants.EncoderUnits / 1) * (Constants.GearboxUnits / 1) * Constants.feetToMeter;
    return (value / rotationsPerMeter);
  }

  public static Drivebase drivebase() {
    return null;
  }

  public void setCoast(boolean b) {
  }
}
