package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
 // TalonSRX is on CAN Bus with device ID 0
    public WPI_PigeonIMU gyro = new WPI_PigeonIMU(3); // Pigeon uses the talon created above
    // The gain for a simple P loop
    //double kP = 0.05;
    private double yawOffset = gyro.getYaw();
    private double rollOffset = gyro.getRoll();
    private double pitchOffset = gyro.getPitch();
    private double angelOffest = gyro.getAngle();

    public void getGyroValues() {
        //Rotation on Ground: Left (Value Increase) - Right (Value Decrease) 
        SmartDashboard.putNumber("Gyro TurnDegree", getYaw());
        //Turn Back and forth: Back (Value Increase) - Forth (Value Decrease)
        SmartDashboard.putNumber("Gyro Roll", getRoll());
        //Turn on side: Left (Value Decrease) - Right (Value Increase) 
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }

    public double getRoll() {
        return gyro.getRoll() - rollOffset;
    }

    public double getYaw() {
        return gyro.getYaw() - yawOffset;
    }

    public double getPitch() {
        return gyro.getPitch() - pitchOffset;
    }

    public double getAngle() {
        return gyro.getAngle() - angelOffest;
    }
}
