// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    // Drivetrain CAN ID's
    public static int klDT1 = 15;
    public static int klDT2 = 14;
    public static int krDT1 = 1;
    public static int krDT2 = 45;

    //DriveTrain Speed
    public static double driveFastRPM = .84;
    public static double driveDefault = .7;
    public static double driveSlowRPM = .4;

    public static double DTleftAxis = 0;
    public static double DTrightAxis = 0;

    //Converter
    public static double EncoderUnits = 2048;
    public static double GearboxUnits = 12.75;
    public static double feetToMeter = 0.479;

    // Claw
    public static int clawR = 0;
    public static int clawL = 0;

    public static int SparkMotorRP = 0;
    public static int SparkMotorLP = 0;
    public static double gearRatio = 0;
    public static boolean motorInverted = true;
    public static double ClawIntakeRPM = -1;
    public static double ClawOuttakeRPM = 1;
    //Wrist
    public static int wrist = 0;

    public static double WristUPRPM = -1;
    public static double WristDOWNRPM = 1;
    //Elevator
    public static int elevatorR = 11;
    public static int elevatorL = 5;
    public static double currentElevatorLimit = 20;
    public static double currentElevatorThreshold = 25;
    public static double currentThresholdTime = 1.0;

    public static double UPelevatorRPM = 1;
    public static double DOWNelevatorRPM = -1;

    //Gyro
    public static int gyro = 0;

    public static int kIndexer = 0;

    public static double Falcon500SensorUnitsConstant = 2048;
    
    public static double kDrivetrainSpeedMultiplier = 0.75;
    public static double kDrivetrainLowSpeedMultiplier = 0.45;

    // true = arcade drive, false = tank drive
    public static boolean isCurvatureDrive = false;
}
