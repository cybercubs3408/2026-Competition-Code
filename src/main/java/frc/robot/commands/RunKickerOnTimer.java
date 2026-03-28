// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakePositionerConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePositionerConstants;
import frc.robot.subsystems.Hopper_Kicker;
import frc.robot.subsystems.IntakePositioner;


public class RunKickerOnTimer extends Command {
  Hopper_Kicker m_hopper_kicker;
  Timer timer = new Timer();


  /** Creates a new Command. */
  public RunKickerOnTimer(Hopper_Kicker hopper_kicker) {
    
    this.m_hopper_kicker = hopper_kicker;
    //means this command will take priority over others using same subsystem
    addRequirements(m_hopper_kicker);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper_kicker.Intake();
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper_kicker.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    
    return (timer.hasElapsed(5.0));
  }
}
