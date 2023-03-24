package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private WPI_TalonFX clawLead = new WPI_TalonFX(Constants.clawL);
 // private WPI_TalonFX clawFollow = new WPI_TalonFX(Constants.clawL);
  private DoubleSolenoid extension = new DoubleSolenoid(Constants.kCompressor, PneumaticsModuleType.CTREPCM, Constants.kExtendedSolenoid, Constants.kRetractedSolenoid);

  // if true, extend. If false, retract
  private boolean extendState = false;

  public Claw(int rightPort, int leftPort, double gearRatio, Boolean invertedLeft) {
    setLeadDefaults();
    // setFollowDefaults();
  }

  @Override
  public void periodic() {
    Value value = extendState ? Value.kForward : Value.kReverse;
    extension.set(value);

    if(clawLead.getMotorOutputPercent() == 0) {
      clawLead.setNeutralMode(NeutralMode.Brake);
      //clawFollow.setNeutralMode(NeutralMode.Brake);
    } else {
      clawLead.setNeutralMode(NeutralMode.Coast);
      //clawFollow.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setLeadDefaults() {
    clawLead.setNeutralMode(NeutralMode.Coast);
    clawLead.configOpenloopRamp(0.25);
    clawLead.setInverted(true);
    clawLead.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    clawLead.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
  }

  // public void setFollowDefaults() {
  //   clawFollow.setNeutralMode(NeutralMode.Coast);
  //   clawFollow.configOpenloopRamp(0.25);
  //   clawFollow.follow(clawLead);
  //   clawFollow.setInverted(InvertType.OpposeMaster);
  // }

  public void setHorizElevatorExtend(boolean extend) {
    extendState = extend;
  }

  public void toggleHorizElevatorExtend() {
    extendState = !extendState;
  }

  public boolean getHorizElevatorExtend() {
    return extendState;
  }

  public void setClawRPM(double RPM) {
    double outputInSensorUnsits = RPM * Constants.Falcon500SensorUnitsConstant / 600;
    clawLead.set(TalonFXControlMode.Velocity, outputInSensorUnsits);
  }

  public void setClawSpeed(double speed) {
    clawLead.set(ControlMode.PercentOutput, speed);
  }

}
/*
 * Falcon Claw Code
 * import com.ctre.phoenix.motorcontrol.NeutralMode;
 * import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
 * import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
 * import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
 * 
 * //claw is motor based intack using falcons
 * //spin in faster, spin out slower
 * public class Claw extends SubsystemBase {
 * WPI_TalonFX clawLead = new WPI_TalonFX(Constants.clawR);
 * WPI_TalonFX clawFollow = new WPI_TalonFX(Constants.clawL);
 * 
 * public Claw() {
 * setClawFollowerDefaults(clawFollow);
 * setClawLeadDefaults(clawLead);
 * clawLead.getSelectedSensorPosition();
 * }
 * 
 * public CommandBase runClaw(double speed) {
 * return runOnce(
 * () -> {
 * setClawRPM(speed);
 * });
 * }
 * 
 * public void setClawRPM(double rpm) {
 * double outputInSensorUnits = rpm * Constants.Falcon500SensorUnitsConstant /
 * 600.0;
 * clawLead.set(TalonFXControlMode.Velocity, outputInSensorUnits);
 * }
 * 
 * private void setClawFollowerDefaults(WPI_TalonFX clawFollow) {
 * clawFollow.configFactoryDefault();
 * clawFollow.setNeutralMode(NeutralMode.Brake);
 * clawFollow.configStatorCurrentLimit(new
 * StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
 * clawFollow.configOpenloopRamp(.25);
 * clawFollow.setInverted(true);
 * clawFollow.follow((clawLead));
 * }
 * 
 * private void setClawLeadDefaults(WPI_TalonFX clawLead) {
 * clawLead.configFactoryDefault();
 * clawLead.setNeutralMode(NeutralMode.Brake);
 * clawLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false,
 * 50, 55, 1.0));
 * clawLead.configOpenloopRamp(.25);
 * clawLead.setInverted(false);
 * }
 * }
 */