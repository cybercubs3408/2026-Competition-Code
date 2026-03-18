// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakePositionerConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePositioner;


public class SetIntakeAngleDown extends Command {
  IntakePositioner m_intake;
  boolean inAuto;


  /** Creates a new Command. */
  public SetIntakeAngleDown(IntakePositioner intake, boolean inAuto) {
    m_intake = intake;
    
    this.inAuto = inAuto;

    //means this command will take priority over others using same subsystem
    addRequirements(m_intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Relative Intake Pos", m_intake.m_IntakePositionerMotor.getAbsoluteEncoder().getPosition());
    

  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakePositioner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    
    return false;
  }
}
