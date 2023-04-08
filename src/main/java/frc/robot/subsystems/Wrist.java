package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Wrist extends SubsystemBase {

    WPI_TalonFX wristTalon = new WPI_TalonFX(Constants.wrist);

    public Wrist() {
        setWristDefaults();
    }

    @Override
    public void periodic() {
        double position = wristTalon.getSelectedSensorPosition();

        SmartDashboard.putNumber("Wrist Position", position);

        boolean limitReached = position > Constants.wrist_Out || position < Constants.wrist_In;
        
        SmartDashboard.putBoolean("Wrist Reached SW Limit", limitReached);
    }

    public void setWristSpeed(double speed) {
        wristTalon.set(TalonFXControlMode.PercentOutput, speed);
    }

    private void setWristDefaults() {
        wristTalon.configFactoryDefault();
        wristTalon.setNeutralMode(NeutralMode.Brake);
        wristTalon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, .25));
        wristTalon.configOpenloopRamp(.05);
        wristTalon.setInverted(false);
        wristTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        wristTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        wristTalon.configReverseSoftLimitThreshold(Constants.wrist_In);
        wristTalon.configReverseSoftLimitEnable(true);
        wristTalon.configForwardSoftLimitThreshold(Constants.wrist_Out);
        wristTalon.configForwardSoftLimitEnable(true);
    }

    public void setWristPosition(double position) {
        if(wristTalon.getSelectedSensorPosition() > position) {
            wristTalon.set(TalonFXControlMode.PercentOutput, -1.0);
        } else {
            wristTalon.set(TalonFXControlMode.PercentOutput, .1);
        }
    }

}