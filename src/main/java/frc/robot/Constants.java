// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    public static final int kPigeon2ID = 13;
    // Hold time on motor brakes when disabled
  }

  public static class ShooterConstants
  {
    public static final int kShooterLeaderID = 14;
    public static final int kShooterFollowerID = 15;
    public static final double kShooterSpeed = 1.0;
    public static final double kShooterBackSpeed = 0.3;
    public static final double kBlueTargetXCoordinate = 4.621;
    public static final double kBlueTargetYCoordinate = 4.029;
    public static final double kRedTargetXCoordinate = 11.919;
    public static final double kRedTargetYCoordinate = 4.029;

    public static final double kLaunchAngle_rad = Math.toRadians(73);
    public static final double kHubtoShooterHeight = 1.2712;
    public static final double kGravity = -9.8;
    public static final double kFlywheelDiameter_meters = 0.1;
    public static final double kSpeedBoostConstant = 0.97;
    public static final double kCenterToShooterDist = 0; //0.229


    //Magic Motion Configs
    public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 0.11; // An error of 1 rps results in 0.11 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0; // no output for error derivative

    public static final double kTargetAcceleration = 400; //RPS^2
    public static final double kTargetJerk = 4000; 
  }

  public static class Hopper_KickerConstants
  {
    public static final int kKickerID = 22;
    public static final int kHopperID = 19;
    public static final double kHopperSpeed = 0.8;
    public static final double kKickerSpeed = 1.0;
    public static final double kKickerSlowSpeed = 0.9;
  }

  public static class IntakePositionerConstants
  {
    public static final int kIntakePositionerID = 18;
    public static final double kIntakePositionerSpeed = 0.15;
    public static final double kLowSpeed = 0;  
    public static final double kSetpoint1 = 0;
    public static final double kSetpoint2 = 6;
    public static final double kSetpoint3 = 16.97;
    
    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.4; // slot 0 by default
    public static final double kV = 0; // slot 0 explicitly
    public static final double kA = 0;
    public static final double kG = 0; // Only use one of kG and kCos
    public static final double kCosRatio = 0;
    public static final double kCruiseVelocity = 5000; //RPM
    public static final double kMaxAcceleration = 5000; 
    
    public static final double kSlowdown = 5.7;
    public static final double kEps_intakePos = 0.7;
  }

  public static class IntakeWheelConstants
  {
    public static final int kIntakeID = 17;
    public static final double kIntakeWheelSpeed = 1.0;
  }

  public static class TurretConstants
  {
    public static final int kTurretID = 16;
    public static final double kTurretSpeed = 0.12;

    public static final double kS = 0.385; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.30; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    public static final double kP = 1.25; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    public static final double kTargetCruiseVelo = 10; //RPS 
    public static final double kTargetAcceleration = 20;
    public static final double kTargetJerk = 0;
    
    public static final double kEps_turret = 0.4;
    public static final double kSlowdown = 5;
    public static final double kRotationToDegreeRatio = 36; 
  }

  public static class ClimberConstants
  {
    public static final int kClimberLeaderID = 22;
    public static final int kClimberFollowerID = 21;
    public static final double kClimberSpeed = 0.6;
    public static final int kServoLeaderID_PWM = 0;
    public static final int kServoFollowerID_PWM = 2;
    public static final double kServoEngage = 1.0;
    public static final double kServoDisengage = 0;

    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0; // slot 0 by default
    public static final double kV = 0; // slot 0 explicitly
    public static final double kA = 0;
    public static final double kG = 0; // Only use one of kG and kCos
    public static final double kCosRatio = 0;
    public static final double kCruiseVelocity = 1000; //RPM
    public static final double kMaxAcceleration = 1000; 

    public static final double kEps_Climber = 2; 
    public static final double kSetpoint1 = 0;
    public static final double kSetpoint2 = 0;
    public static final double kSetpoint3 = 0;
  }
}
