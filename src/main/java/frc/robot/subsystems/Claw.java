package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private CANSparkMax rightMotor, leftMotor;

    

    public Claw(int rightPort, int leftPort, double gearRatio, Boolean invertedLeft) {

        rightMotor = new CANSparkMax(leftPort, MotorType.kBrushless);
        leftMotor = new CANSparkMax(rightPort, MotorType.kBrushless);

        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();

        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);

        if (invertedLeft) {
			leftMotor.setInverted(true);
		} else {
			rightMotor.setInverted(true);
		}
        
        leftMotor.follow(rightMotor);
    }
        public CommandBase runClaw(double speed) {
            return runOnce(
              () -> {
                    rightMotor.set(speed);
                });
           }
      
      }
/* Falcon Claw Code
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//claw is motor based intack using falcons
//spin in faster, spin out slower 
public class Claw extends SubsystemBase {
    WPI_TalonFX clawLead = new WPI_TalonFX(Constants.clawR);
    WPI_TalonFX clawFollow = new WPI_TalonFX(Constants.clawL);

    public Claw() {
        setClawFollowerDefaults(clawFollow);
        setClawLeadDefaults(clawLead);
        clawLead.getSelectedSensorPosition();
    }

    public CommandBase runClaw(double speed) {
        return runOnce(
          () -> {
              setClawRPM(speed);
          });
    }
    
  public void setClawRPM(double rpm) {
    double outputInSensorUnits = rpm * Constants.Falcon500SensorUnitsConstant / 600.0;
    clawLead.set(TalonFXControlMode.Velocity, outputInSensorUnits);
  }

    private void setClawFollowerDefaults(WPI_TalonFX clawFollow) {
        clawFollow.configFactoryDefault();
        clawFollow.setNeutralMode(NeutralMode.Brake);
        clawFollow.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
        clawFollow.configOpenloopRamp(.25);
        clawFollow.setInverted(true);
        clawFollow.follow((clawLead));
    }

    private void setClawLeadDefaults(WPI_TalonFX clawLead) {
        clawLead.configFactoryDefault();
        clawLead.setNeutralMode(NeutralMode.Brake);
        clawLead.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));
        clawLead.configOpenloopRamp(.25);
        clawLead.setInverted(false);
    }
}
*/