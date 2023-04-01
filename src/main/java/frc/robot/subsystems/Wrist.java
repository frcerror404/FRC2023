package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Wrist extends SubsystemBase {

    WPI_TalonFX wristTalon = new WPI_TalonFX(Constants.wrist);

    public Wrist() {
        setWristDefaults();
    }

    public void setWristSpeed(double speed) {
        wristTalon.set(TalonFXControlMode.PercentOutput, speed);
    }

    private void setWristDefaults() {
        wristTalon.configFactoryDefault();
        wristTalon.setNeutralMode(NeutralMode.Brake);
        wristTalon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 15, 20, .25));
        wristTalon.configOpenloopRamp(.05);
        wristTalon.setInverted(false);

        wristTalon.configForwardSoftLimitThreshold(Constants.wrist_Out);
        wristTalon.configReverseSoftLimitThreshold(Constants.wrist_In);
        wristTalon.configForwardSoftLimitEnable(true);
        wristTalon.configReverseSoftLimitEnable(true);
    }

    public void setWristPosition(double position) {
        if(wristTalon.getSelectedSensorPosition() > position) {
            wristTalon.set(TalonFXControlMode.PercentOutput, -1.0);
        } else {
            wristTalon.set(TalonFXControlMode.PercentOutput, .1);
        }
    }

}