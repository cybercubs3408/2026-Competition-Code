// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterSpeed extends Command {
  /** Creates a new FindShootingAngle. */
  CommandSwerveDrivetrain m_drivetrain;
  Shooter m_shooter;
  boolean inAuto;


  public SetShooterSpeed(CommandSwerveDrivetrain drivetrain, Shooter shooter, boolean inAuto) 
  {
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    
    this.inAuto = inAuto;

    //means this command will take priority over others using same subsystem
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    var allianceColor = DriverStation.getAlliance();
    var currentXPos = m_drivetrain.getState().Pose.getX();
    var currentYPos = m_drivetrain.getState().Pose.getY();
    double distance = 0; 

    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        distance = Math.sqrt(Math.pow((ShooterConstants.kBlueTargetYCoordinate-currentYPos),2)+Math.pow((ShooterConstants.kBlueTargetXCoordinate-currentXPos),2));
      } else {
        distance = Math.sqrt(Math.pow((currentYPos-ShooterConstants.kRedTargetYCoordinate),2)+Math.pow((currentXPos-ShooterConstants.kRedTargetXCoordinate),2));
      }
    }



    double d1 = 2*Math.pow((Math.cos(ShooterConstants.kLaunchAngle_rad)),2);
    double d2 = ShooterConstants.kHubtoShooterHeight-(Math.tan(ShooterConstants.kLaunchAngle_rad)*distance);
    double n = (ShooterConstants.kGravity*Math.pow(distance,2));
    double initialVelo = Math.sqrt((n)/(d1*d2));
    double targetRPS = ShooterConstants.kSpeedBoostConstant*(initialVelo*2)/(Math.PI*ShooterConstants.kFlywheelDiameter_meters);

    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    // set target velocity to 80 rps
    m_shooter.m_shooterLeader.setControl(m_request.withVelocity(targetRPS));
    System.out.println(targetRPS);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_shooter.m_shooterLeader.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_shooter.m_shooterLeader.getMotionMagicAtTarget().getValue() && inAuto);
  }
}
