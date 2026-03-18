// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;


public class SetClimberPosition extends Command {
  Climber m_climber;
  double setpoint;
  boolean inAuto;


  /** Creates a new Command. */
  public SetClimberPosition(Climber climber, double setpoint, boolean inAuto) {
    m_climber = climber;
    
    this.setpoint = setpoint;
    this.inAuto = inAuto;

    //means this command will take priority over others using same subsystem
    addRequirements(m_climber);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.closedLoopController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    
    return ((m_climber.m_ClimberLeader.getAbsoluteEncoder().getPosition() < setpoint + ClimberConstants.kEps_Climber)&&(m_climber.m_ClimberLeader.getAbsoluteEncoder().getPosition() > setpoint - ClimberConstants.kEps_Climber)&& inAuto);
  }
}
