// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.Timer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShootingConditions extends Command {
  /** Creates a new FindShootingAngle. */
  CommandSwerveDrivetrain drivetrain;
  Turret m_turret;
  Shooter m_shooter;
  double targetAngle = 0;
  double robotAngle;
  double fieldAngle;
  double motorTarget;
  boolean useConditional;
  boolean useTimer;
  double currPos_deg;
  double targetRotations;
  Timer timer = new Timer();


  public SetShootingConditions(CommandSwerveDrivetrain drivetrain, Turret turret, Shooter shooter, boolean useConditional, boolean useTimer) 
  {
    m_turret = turret;
    m_shooter = shooter;
    this.drivetrain = drivetrain;
    
    this.useConditional = useConditional;
    this.useTimer = useTimer;

    //means this command will take priority over others using same subsystem
    addRequirements(m_turret);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    var allianceColor = DriverStation.getAlliance();
    var currentXPos_r = drivetrain.getState().Pose.getX();
    var currentYPos_r = drivetrain.getState().Pose.getY();
    
    robotAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    double robotAngle_turretOrigin = robotAngle;

    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        if (robotAngle > 0)
        {
          robotAngle_turretOrigin = robotAngle - 180;
        }
        else
        {
          robotAngle_turretOrigin = robotAngle + 180;
        }
      }
    }


    var currentXPos_s = currentXPos_r + ShooterConstants.kCenterToShooterDist*Math.cos(robotAngle_turretOrigin);
    var currentYPos_s = currentYPos_r + ShooterConstants.kCenterToShooterDist*Math.sin(robotAngle_turretOrigin);

    var xPos_hubOrigin_blue = currentXPos_s-ShooterConstants.kBlueTargetXCoordinate;
    var yPos_hubOrigin_blue = currentYPos_s-ShooterConstants.kBlueTargetYCoordinate;
    var xPos_hubOrigin_red = currentXPos_s-ShooterConstants.kRedTargetXCoordinate;
    var yPos_hubOrigin_red = currentYPos_s-ShooterConstants.kRedTargetYCoordinate;

    currPos_deg = m_turret.m_turretMotor.getPosition().getValueAsDouble()*TurretConstants.kRotationToDegreeRatio;
    
    
    
    System.out.println("xPos " + xPos_hubOrigin_red);
    System.out.println("yPos " + yPos_hubOrigin_red);
    
    System.out.println("turret pos " + currPos_deg);
    System.out.println("robot angle " + robotAngle);


    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        fieldAngle = Math.toDegrees(Math.atan2(-xPos_hubOrigin_blue, yPos_hubOrigin_blue));
      } else {
        fieldAngle = Math.toDegrees(Math.atan2(xPos_hubOrigin_red, yPos_hubOrigin_red));
      }
    }
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    
    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        targetAngle = 90-fieldAngle + robotAngle_turretOrigin;
      } else {
        targetAngle = fieldAngle-90 + robotAngle_turretOrigin;
      }
    }

    if ((targetAngle > 95) || (targetAngle < -95))
    {
      targetAngle = 0;
    }
    
    targetRotations = targetAngle/TurretConstants.kRotationToDegreeRatio;

    m_turret.m_turretMotor.setControl(m_request.withPosition(targetRotations));

    //SHOOTER
    double distance = 0; 

    if (allianceColor.isPresent()) {
      //blue alliance targets
      if (allianceColor.get() == Alliance.Blue) {
        distance = Math.sqrt(Math.pow(xPos_hubOrigin_blue,2)+Math.pow(yPos_hubOrigin_blue,2));
      } else {
        distance = Math.sqrt(Math.pow(xPos_hubOrigin_red,2)+Math.pow(yPos_hubOrigin_red,2));
      }
    }



    double d1 = 2*Math.pow((Math.cos(ShooterConstants.kLaunchAngle_rad)),2);
    double d2 = ShooterConstants.kHubtoShooterHeight-(Math.tan(ShooterConstants.kLaunchAngle_rad)*distance);
    double n = (ShooterConstants.kGravity*Math.pow(distance,2));
    double initialVelo = Math.sqrt((n)/(d1*d2));
    double targetRPS = ShooterConstants.kSpeedBoostConstant*(initialVelo*2)/(Math.PI*ShooterConstants.kFlywheelDiameter_meters);

    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_requestShooter = new MotionMagicVelocityVoltage(0);

    // set target velocity to 80 rps
    m_shooter.m_shooterLeader.setControl(m_requestShooter.withVelocity(targetRPS));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_turret.stopTurret();
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_turret.m_turretMotor.getMotionMagicAtTarget().getValue() 
      && m_shooter.m_shooterLeader.getMotionMagicAtTarget().getValue() 
      && useConditional && (useTimer && (timer.hasElapsed(3.0))));
    //return ((currPos < targetAngle + TurretConstants.kEps_turret)&&(currPos > targetAngle - TurretConstants.kEps_turret)&& useConditional);
  }
}
