// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Shooter extends SubsystemBase {
  /** Creates a new ShooterWheels. */
  //public TalonFX m_shooterFollower;
  public TalonFX m_shooterLeader;
  public TalonFX m_shooterFollower;

  public Shooter() 
  {
    m_shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerID);
    m_shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderID);

    //right is the leader, left is the follower
    m_shooterFollower.setControl(new Follower(m_shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = ShooterConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ShooterConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = ShooterConstants.kI; // no output for integrated error
    slot0Configs.kD = ShooterConstants.kD; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.kTargetAcceleration; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = ShooterConstants.kTargetJerk; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_shooterLeader.getConfigurator().apply(talonFXConfigs);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */

  public Command Shoot() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setShooterSpeed(ShooterConstants.kShooterSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopShooter();
        });
  }

  public Command ShootBack() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setShooterSpeed(ShooterConstants.kShooterBackSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopShooter();
        });
  }

  public Command MagicMotionShoot(double rotPerSec) {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
          m_shooterLeader.setControl(m_request.withVelocity(rotPerSec));
        },
        // When the command stops, stop the wheels
        () -> {
          stopShooter();
        });
  }



  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setShooterSpeed(double speed) {
    m_shooterLeader.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopShooter() {
    m_shooterLeader.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
