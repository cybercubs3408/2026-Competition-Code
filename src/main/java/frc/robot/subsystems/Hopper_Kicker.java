// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import frc.robot.Constants.Hopper_KickerConstants;

public class Hopper_Kicker extends SubsystemBase {
  /** Creates a new Hopper. */
  public SparkMax m_Hopper;
  public SparkMax m_Kicker;

  public Hopper_Kicker() 
  {
    m_Hopper = new SparkMax(Hopper_KickerConstants.kHopperID, MotorType.kBrushless);
    m_Kicker = new SparkMax(Hopper_KickerConstants.kKickerID, MotorType.kBrushless);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */

  public Command Intake() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setSpeeds(-Hopper_KickerConstants.kHopperSpeed, -Hopper_KickerConstants.kKickerSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  public Command Outake() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setSpeeds(Hopper_KickerConstants.kHopperSpeed, Hopper_KickerConstants.kKickerSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

    public Command runHopper() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          m_Hopper.set(0.7);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setSpeeds(double HopperSpeed, double KickerSpeed) {
    m_Hopper.set(HopperSpeed);
    m_Kicker.set(KickerSpeed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_Hopper.set(0);
    m_Kicker.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
