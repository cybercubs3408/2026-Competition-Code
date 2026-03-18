// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;

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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTurretPosition extends Command {
  /** Creates a new FindShootingAngle. */
  CommandSwerveDrivetrain drivetrain;
  Turret m_turret;
  double targetAngle;
  double motorTarget;
  double fieldAngle;
  double robotAngle;
  double targetRotations;
  double currPos_deg;
  boolean inAuto;


  public SetTurretPosition(CommandSwerveDrivetrain drivetrain, Turret turret, boolean inAuto) 
  {
    m_turret = turret;
    
    this.inAuto = inAuto;
    this.drivetrain = drivetrain;

    //means this command will take priority over others using same subsystem
    addRequirements(m_turret);
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
    var currentXPos_r = drivetrain.getState().Pose.getX();
    var currentYPos_r = drivetrain.getState().Pose.getY();
    
    robotAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    double robotAngle_turretOrigin = 0;

    if (robotAngle > 0)
    {
      robotAngle_turretOrigin = robotAngle - 180;
    }
    else
    {
      robotAngle_turretOrigin = robotAngle + 180;
    }


    var currentXPos_s = currentXPos_r + ShooterConstants.kCenterToShooterDist*Math.cos(robotAngle_turretOrigin);
    var currentYPos_s = currentYPos_r + ShooterConstants.kCenterToShooterDist*Math.sin(robotAngle_turretOrigin);

    var xPos_hubOrigin_blue = currentXPos_s-ShooterConstants.kBlueTargetXCoordinate;
    var yPos_hubOrigin_blue = currentYPos_s-ShooterConstants.kBlueTargetYCoordinate;
    var xPos_hubOrigin_red = currentXPos_s-ShooterConstants.kRedTargetXCoordinate;
    var yPos_hubOrigin_red = currentYPos_s-ShooterConstants.kRedTargetYCoordinate;

    currPos_deg = m_turret.m_turretMotor.getPosition().getValueAsDouble()*TurretConstants.kRotationToDegreeRatio;
    
    
    
    System.out.println("xPos " + xPos_hubOrigin_blue);
    System.out.println("yPos " + yPos_hubOrigin_blue);
    
    System.out.println("turret pos " + currPos_deg);
    System.out.println("robot angle " + robotAngle);


    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_blue, yPos_hubOrigin_blue));
      } else {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_red, yPos_hubOrigin_red));
      }
    }
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    targetAngle = 90-fieldAngle + robotAngle_turretOrigin;

    if ((targetAngle > 95) || (targetAngle < -95))
    {
      targetAngle = 0;
    }
    
    targetRotations = targetAngle/TurretConstants.kRotationToDegreeRatio;

    m_turret.m_turretMotor.setControl(m_request.withPosition(targetRotations));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_turret.m_turretMotor.getMotionMagicAtTarget().getValue() && inAuto);
    //return ((currPos < targetAngle + TurretConstants.kEps_turret)&&(currPos > targetAngle - TurretConstants.kEps_turret)&& inAuto);
  }
}


/*    //MAKE MOTOR LIMIT SWITCH IN PHOENIX TUNER X TO ASSIST IN PREVENTING OVERROTATION
    var allianceColor = DriverStation.getAlliance();
    var currentXPos_r = drivetrain.getState().Pose.getX();
    var currentYPos_r = drivetrain.getState().Pose.getY();
    
    robotAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    var robotAngle_turretOrigin;

    if (robotAngle > 0)
    {
      robotAngle_turretOrigin = robotAngle - 180;
    }
    else
    {
      robotAngle_turretOrigin = robotAngle + 180;
    }


    var currentXPos_s = currentXPos_r + ShooterConstants.kCenterToShooterDist*Math.cos(robotAngle);
    var currentYPos_s = currentYPos_r + ShooterConstants.kCenterToShooterDist*Math.sin(robotAngle);

    var xPos_hubOrigin_blue = currentXPos_s-ShooterConstants.kBlueTargetXCoordinate;
    var yPos_hubOrigin_blue = currentYPos_s-ShooterConstants.kBlueTargetYCoordinate;
    var xPos_hubOrigin_red = currentXPos_s-ShooterConstants.kRedTargetXCoordinate;
    var yPos_hubOrigin_red = currentYPos_s-ShooterConstants.kRedTargetYCoordinate;

    currPos_deg = m_turret.m_turretMotor.getPosition().getValueAsDouble()*TurretConstants.kRotationToDegreeRatio;
    
    
    
    System.out.println("xPos " + xPos_hubOrigin_blue);
    System.out.println("yPos " + yPos_hubOrigin_blue);
    
    System.out.println("turret pos " + currPos_deg);
    System.out.println("robot angle " + robotAngle);


    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_blue, yPos_hubOrigin_blue));
      } else {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_red, yPos_hubOrigin_red));
      }
    }
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    targetAngle = 90-fieldAngle + robotAngle;

    if ((targetAngle > 95) || (targetAngle < -95))
    {
      targetAngle = 0;
    }
    
    targetRotations = targetAngle/TurretConstants.kRotationToDegreeRatio;

    m_turret.m_turretMotor.setControl(m_request.withPosition(targetRotations)); */

//THIS VERSION WORKS WITHOUT ADDING ROBOT ROTATION
/*    //MAKE MOTOR LIMIT SWITCH IN PHOENIX TUNER X TO ASSIST IN PREVENTING OVERROTATION
    var allianceColor = DriverStation.getAlliance();
    var currentXPos_r = drivetrain.getState().Pose.getX();
    var currentYPos_r = drivetrain.getState().Pose.getY();
    robotAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    
    var currentXPos_s = currentXPos_r; //+ ShooterConstants.kCenterToShooterDist*Math.cos(robotAngle);
    var currentYPos_s = currentYPos_r; //+ ShooterConstants.kCenterToShooterDist*Math.sin(robotAngle);

    var xPos_hubOrigin_blue = currentXPos_s-ShooterConstants.kBlueTargetXCoordinate;
    var yPos_hubOrigin_blue = currentYPos_s-ShooterConstants.kBlueTargetYCoordinate;
    var xPos_hubOrigin_red = currentXPos_s-ShooterConstants.kRedTargetXCoordinate;
    var yPos_hubOrigin_red = currentYPos_s-ShooterConstants.kRedTargetYCoordinate;

    currPos_deg = m_turret.m_turretMotor.getPosition().getValueAsDouble()*TurretConstants.kRotationToDegreeRatio;
    
    
    System.out.println("xPos " + xPos_hubOrigin_blue);
    System.out.println("yPos " + yPos_hubOrigin_blue);
    
    System.out.println("turret pos " + currPos_deg);
    System.out.println("robot angle " + robotAngle);


    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_blue + ShooterConstants.kCenterToShooterDist, yPos_hubOrigin_blue));
      } else {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_red, yPos_hubOrigin_red));
      }
    }
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    targetAngle = 90-fieldAngle;


    if ((targetAngle > 95) || (targetAngle < -95))
    {
      targetAngle = 0;
    }
    
    targetRotations = targetAngle/TurretConstants.kRotationToDegreeRatio;

    m_turret.m_turretMotor.setControl(m_request.withPosition(targetRotations)); */