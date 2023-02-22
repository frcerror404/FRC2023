package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final WPI_TalonFX elevatorLead = new WPI_TalonFX(Constants.elevatorL);
    private final WPI_TalonFX elevatorFollow = new WPI_TalonFX(Constants.elevatorR);
    
    public Elevator() {
        setLeadDefaults();
        setFollowDefault();
    }

    public CommandBase runElevator(double speed) {
        return runOnce(
          () -> {
              setElevatorRPM(speed);
          });
    }

    public void setElevatorRPM(double rpm) {
        double outputInSensorUnits = rpm * Constants.Falcon500SensorUnitsConstant / 600.0;
        elevatorLead.set(TalonFXControlMode.Velocity, outputInSensorUnits);
      }

    public void setLeadDefaults() {
        elevatorLead.configFactoryDefault();
        elevatorLead.setNeutralMode(NeutralMode.Coast);
        elevatorLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.currentElevatorLimit, Constants.currentElevatorThreshold, Constants.currentThresholdTime));
        elevatorLead.configOpenloopRamp(.25);
        elevatorLead.setInverted(false);
    }

    public void setFollowDefault() {
        elevatorFollow.configFactoryDefault();
        elevatorFollow.setNeutralMode(NeutralMode.Coast);
        elevatorFollow.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.currentElevatorLimit, Constants.currentElevatorThreshold, Constants.currentThresholdTime));
        elevatorFollow.configOpenloopRamp(.25);
        elevatorFollow.setInverted(true);
        elevatorFollow.follow(elevatorLead);
    }

    public void getSensorPositios() {
        SmartDashboard.putNumber("Elevator Lead Sensor", elevatorLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elevator Follow Sensor", elevatorFollow.getSelectedSensorPosition());
    }
}
