// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakePositionerConstants;

public class IntakePositioner extends SubsystemBase {
  /** Creates a new IntakePositioner. */
  public SparkMax m_IntakePositionerMotor;
  public SparkMaxConfig motorConfig;
  public SparkClosedLoopController closedLoopController;
  public RelativeEncoder encoder;
  public DigitalInput m_intakeLimitSwitch;

  public IntakePositioner() 
  {
    m_IntakePositionerMotor = new SparkMax(IntakePositionerConstants.kIntakePositionerID, MotorType.kBrushless);
    closedLoopController = m_IntakePositionerMotor.getClosedLoopController();
    encoder = m_IntakePositionerMotor.getEncoder();
    m_intakeLimitSwitch = new DigitalInput(0);

        /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
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

    motorConfig.closedLoop.maxMotion
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
    m_IntakePositionerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */

  public Command IntakePosUp(double speed) {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setIntakePositionerSpeed(-speed);
          System.out.println(m_IntakePositionerMotor.getEncoder().getPosition());
        },
        // When the command stops, stop the wheels
        () -> {
          stopIntakePositioner();
        });
  }

  public Command IntakePosDown(double speed) {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setIntakePositionerSpeed(speed);
          System.out.println(m_IntakePositionerMotor.getEncoder().getPosition());
        },
        // When the command stops, stop the wheels
        () -> {
          stopIntakePositioner();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setIntakePositionerSpeed(double speed) {
    m_IntakePositionerMotor.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopIntakePositioner() {
    m_IntakePositionerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
