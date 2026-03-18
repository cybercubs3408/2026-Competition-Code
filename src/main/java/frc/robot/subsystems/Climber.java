// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakePositionerConstants;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax m_ClimberFollower;
  public SparkMax m_ClimberLeader;
  public SparkMaxConfig leaderMotorConfig;
  public SparkMaxConfig followerMotorConfig; 
  public SparkClosedLoopController closedLoopController;
  public RelativeEncoder encoder;

  public Servo m_ratchetServoLeader;
  public Servo m_ratchetServoFollower;

  public boolean ratchetsEngaged;

  public Climber() 
  {
    //MAKE MOTOR LIMIT SWITCH IN PHOENIX TUNER X TO PREVENT OVERROTATION
    m_ClimberFollower = new SparkMax(ClimberConstants.kClimberFollowerID, MotorType.kBrushless);
    m_ClimberLeader = new SparkMax(ClimberConstants.kClimberLeaderID, MotorType.kBrushless);

    m_ratchetServoLeader = new Servo(ClimberConstants.kServoLeaderID_PWM);
    m_ratchetServoLeader.setBoundsMicroseconds(2500,2300,1500,700,500);

    m_ratchetServoFollower = new Servo(ClimberConstants.kServoFollowerID_PWM);
    m_ratchetServoFollower.setBoundsMicroseconds(2500,2300,1500,700,500);

    setServos(ClimberConstants.kServoDisengage);
    ratchetsEngaged = false; 


    closedLoopController = m_ClimberLeader.getClosedLoopController();
    encoder = m_ClimberLeader.getEncoder();

        /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    leaderMotorConfig = new SparkMaxConfig();
    followerMotorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    leaderMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    followerMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    followerMotorConfig
      .follow(m_ClimberLeader);


    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    leaderMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(IntakePositionerConstants.kP)
        .i(IntakePositionerConstants.kI)
        .d(IntakePositionerConstants.kD)
        .outputRange(-1, 1)
        .feedForward
            .kS(IntakePositionerConstants.kS) // slot 0 by default
            .kV(IntakePositionerConstants.kV, ClosedLoopSlot.kSlot0) // slot 0 explicitly
            .kA(IntakePositionerConstants.kA)
            .kG(IntakePositionerConstants.kG) // Only use one of kG and kCos
            .kCosRatio(IntakePositionerConstants.kCosRatio);

    leaderMotorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(IntakePositionerConstants.kCruiseVelocity)
        .maxAcceleration(IntakePositionerConstants.kMaxAcceleration)
        .allowedProfileError(1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_ClimberLeader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_ClimberFollower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */

  public Command ClimbForward() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setClimberSpeed(ClimberConstants.kClimberSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopClimber();
        });
  }

  public Command ClimbBackward() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setClimberSpeed(-ClimberConstants.kClimberSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopClimber();
        });
  }

  public Command ClimbToSetpoint(double setpoint) {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          closedLoopController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
        },
        // When the command stops, stop the wheels
        () -> {
          
        });
  }

  public Command DisengageServo() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        () -> {
          setServos(ClimberConstants.kServoDisengage);
        },
        () -> {
          
        });
  }

    public Command EngageServo() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        () -> {
            setServos(ClimberConstants.kServoEngage);
        },
        () -> {
          
        });
  }

      public Command SetServoEngagement() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        () -> {
          if (ratchetsEngaged)
          {
            setServos(ClimberConstants.kServoDisengage);
          }
          else
          {
            setServos(ClimberConstants.kServoEngage);
          }
        },
        () -> {
          
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setClimberSpeed(double speed) {
    m_ClimberLeader.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopClimber() {
    m_ClimberLeader.set(0);
  }

  public void setServos(double speed)
  {
    m_ratchetServoLeader.setPosition(speed);
    m_ratchetServoFollower.setPosition(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
