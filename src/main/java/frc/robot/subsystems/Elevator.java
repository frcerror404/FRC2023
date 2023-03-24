package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final WPI_TalonFX elevatorLead = new WPI_TalonFX(Constants.elevatorL);
    private final WPI_TalonFX elevatorFollow = new WPI_TalonFX(Constants.elevatorR);
    private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    public Elevator() {
        setLeadDefaults();
        //setFollowDefault();
    }

    public void setElevatorSpeed(double speed) {


        if(getElevatorPosition() < Constants.ele_UpperLimit *.7 && speed > 0) {
            speed *= .5;
        }
        if(getElevatorPosition() > Constants.ele_LowerLimit * .7 && speed < 0) {
            speed *= .5;
        }

        elevatorLead.set(ControlMode.PercentOutput, -speed);
    }

    public void setElevatorRPM(double rpm) {
        double outputInSensorUnits = rpm * Constants.Falcon500SensorUnitsConstant / 600.0;
        elevatorLead.set(TalonFXControlMode.Velocity, outputInSensorUnits);
    }

    public void setLeadDefaults() {
        //elevatorLead.configFactoryDefault();
        elevatorLead.setNeutralMode(NeutralMode.Coast);
        //elevatorLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.currentElevatorLimit,
        //        Constants.currentElevatorThreshold, Constants.currentThresholdTime));
        //elevatorLead.configOpenloopRamp(.75);
        elevatorLead.setInverted(false);

        elevatorLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 2, 0.5));
        elevatorConfig.slot0.kP = .002;
        elevatorConfig.slot0.kI = 0;
        elevatorConfig.slot0.kF = 0;
        elevatorConfig.slot0.integralZone = 0;
        elevatorConfig.slot0.allowableClosedloopError = 0;
        elevatorConfig.slot0.closedLoopPeriod = 10;
        elevatorConfig.slot0.maxIntegralAccumulator = 0;
        elevatorConfig.slot0.closedLoopPeakOutput = 1;
        
        elevatorLead.configAllSettings(elevatorConfig);

        elevatorLead.configReverseSoftLimitEnable(true);
        elevatorLead.configReverseSoftLimitThreshold(Constants.ele_UpperLimit);
        elevatorLead.configForwardSoftLimitEnable(true);
        elevatorLead.configForwardSoftLimitThreshold(Constants.ele_LowerLimit);

        elevatorFollow.follow(elevatorLead);
        elevatorFollow.setNeutralMode(NeutralMode.Coast);
        elevatorFollow.setInverted(InvertType.OpposeMaster);
        //elevatorFollow.configOpenloopRamp(.75);
    }

    //public void setFollowDefault() {
    //    //elevatorFollow.configFactoryDefault();
    //    elevatorFollow.setNeutralMode(NeutralMode.Coast);
    //    elevatorFollow.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
    //            Constants.currentElevatorLimit, Constants.currentElevatorThreshold, Constants.currentThresholdTime));
    //    elevatorFollow.configOpenloopRamp(.05);
    //    elevatorFollow.follow(elevatorLead);
    //    elevatorFollow.setInverted(InvertType.OpposeMaster);
    //    
    //}

    public void getSensorPositios() {
        SmartDashboard.putNumber("Elevator Lead Sensor", elevatorLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elevator Follow Sensor", elevatorFollow.getSelectedSensorPosition());
    }

    public double getElevatorPosition() {
        return elevatorLead.getSelectedSensorPosition();
    }

    public void setElevatorPosition(double units) {
        elevatorLead.set(ControlMode.Position, units);
    }
}
