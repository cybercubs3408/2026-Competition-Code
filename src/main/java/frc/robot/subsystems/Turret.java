// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public TalonFX m_turretMotor;

  public Turret() 
  {
    m_turretMotor = new TalonFX(TurretConstants.kTurretID);
    m_turretMotor.setPosition(0);

    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = TurretConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = TurretConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = TurretConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = TurretConstants.kP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = TurretConstants.kI; // no output for integrated error
    slot0Configs.kD = TurretConstants.kD; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = TurretConstants.kTargetCruiseVelo; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = TurretConstants.kTargetAcceleration; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = TurretConstants.kTargetJerk; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_turretMotor.getConfigurator().apply(talonFXConfigs);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */
  public Command OutputTurretPos() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          System.out.println(m_turretMotor.getPosition().getValueAsDouble());
        },
        // When the command stops, stop the wheels
        () -> {
          stopTurret();
        });
  }

  public Command TurrentClockwise() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setTurretSpeed(TurretConstants.kTurretSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopTurret();
        });
  }

  public Command TurretCounterClockwise() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setTurretSpeed(-TurretConstants.kTurretSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopTurret();
        });
  }


    public Command GoToPos() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    
    m_turretMotor.setControl(m_request.withPosition(4));
    System.out.println(m_turretMotor.getPosition().getValueAsDouble());
        },
        // When the command stops, stop the wheels
        () -> {
          stopTurret();
        });
  }
  
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setTurretSpeed(double speed) {
    m_turretMotor.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopTurret() {
    m_turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
