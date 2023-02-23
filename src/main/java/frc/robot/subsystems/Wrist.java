package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Wrist extends SubsystemBase {

    WPI_TalonFX wrisTalonFX = new WPI_TalonFX(Constants.wrist);

    public Wrist() {
        setWristDefaults(wrisTalonFX);
        wrisTalonFX.getSelectedSensorPosition();

    }

    public CommandBase moveWrist(double speed) {
        return runOnce(
                () -> {
                    setWristRPM(speed);
                });
    }

    public void setWristRPM(double rpm) {
        double outputInSensorUnits = rpm * Constants.Falcon500SensorUnitsConstant / 600.0;
        wrisTalonFX.set(TalonFXControlMode.Velocity, outputInSensorUnits);
    }

    private void setWristDefaults(WPI_TalonFX wrisTalonFX) {
        wrisTalonFX.configFactoryDefault();
        wrisTalonFX.setNeutralMode(NeutralMode.Brake);
        wrisTalonFX.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
        wrisTalonFX.configOpenloopRamp(.25);
        wrisTalonFX.setInverted(false);
    }

}