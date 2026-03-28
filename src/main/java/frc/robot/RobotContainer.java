// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.Hopper_KickerConstants;
import frc.robot.Constants.IntakePositionerConstants;
import frc.robot.commands.SetIntakeAngle_MAXMotion;
import frc.robot.commands.SetShootingConditions;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeWheel;
import frc.robot.subsystems.Hopper_Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IntakePositioner;
import frc.robot.commands.SetTurretPosition;


public class RobotContainer {
    Shooter m_shooter = new Shooter();
    Hopper_Kicker m_hopper_kicker = new Hopper_Kicker();
    IntakeWheel m_intakeWheel = new IntakeWheel();
    Turret m_turret = new Turret();
    IntakePositioner m_intakePositioner = new IntakePositioner();
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController mechXbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //NAMED COMMANDS HERE FOR PATHPLANNER

        //need to figure out how to keep flywheel running after command ends
        NamedCommands.registerCommand("SetShootingConditions", new SetShootingConditions(drivetrain, m_turret, m_shooter, false, false));
        NamedCommands.registerCommand("SetShootingConditions_noTimer", new SetShootingConditions(drivetrain, m_turret, m_shooter, false, false));
        NamedCommands.registerCommand("FeedToShooter", m_hopper_kicker.Intake());
        NamedCommands.registerCommand("DeployIntake", new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint3, false));
        NamedCommands.registerCommand("DeployIntake_Halfway", new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint2, false));
        NamedCommands.registerCommand("IntakeBalls", m_intakeWheel.Intake());
        NamedCommands.registerCommand("FeedToShooter_onTimer", m_hopper_kicker.Intake());
        /* 
        NamedCommands.registerCommand("SetClimber_Pos2", m_climber.ClimbToSetpoint(ClimberConstants.kSetpoint2));
        NamedCommands.registerCommand("SetClimber_Pos3", m_climber.ClimbToSetpoint(ClimberConstants.kSetpoint3));
        NamedCommands.registerCommand("EngageRatchets", m_climber.EngageServo());
        NamedCommands.registerCommand("DisengageRatchets", m_climber.DisengageServo());
        */
        autoChooser = AutoBuilder.buildAutoChooser("Auto Paths");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        /*
        driverXbox.b().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        ));
         */
        // Run SysId routines when holding back/start and povDown/povUp.
        // Note that each routine should be run exactly once in a single log.
        driverXbox.back().and(driverXbox.povUp()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverXbox.back().and(driverXbox.povDown()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverXbox.start().and(driverXbox.povUp()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverXbox.start().and(driverXbox.povDown()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //driverXbox.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        
        //these are just for testing
        driverXbox.x().onTrue(m_turret.GoToPos(0));
        driverXbox.rightTrigger().whileTrue(new SetShootingConditions(drivetrain, m_turret, m_shooter, false, false));
        driverXbox.rightBumper().whileTrue(Commands.parallel(m_turret.GoToPos(0), m_shooter.MagicMotionShoot(50)));
        //driverXbox.rightBumper().whileTrue(new SetShooterSpeed(drivetrain, m_shooter, false));
        driverXbox.leftTrigger().whileTrue(m_hopper_kicker.Intake());
        //driverXbox.leftBumper().whileTrue(new SetTurretPosition(drivetrain, m_turret, false));
        driverXbox.leftBumper().whileTrue(m_intakeWheel.Intake());
        //driverXbox.povLeft().whileTrue(m_turret.TurretCounterClockwise());
        //driverXbox.povRight().whileTrue(m_turret.TurrentClockwise());
        
        driverXbox.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverXbox.y().whileTrue(m_intakePositioner.IntakePosUp(IntakePositionerConstants.kIntakePositionerSpeed));
        driverXbox.a().whileTrue(m_intakePositioner.IntakePosDown(IntakePositionerConstants.kIntakePositionerSpeed));        
        driverXbox.back().whileTrue(m_hopper_kicker.Outake());
        driverXbox.povUp().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint1, false));
        driverXbox.povRight().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint2, false));
        driverXbox.povDown().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint3, false));



        mechXbox.x().whileTrue(m_turret.TurretCounterClockwise());
        mechXbox.b().whileTrue(m_turret.TurrentClockwise());
        //mechXbox.leftTrigger().whileTrue(m_hopper_kicker.Intake());
        mechXbox.leftBumper().whileTrue(m_intakeWheel.Intake());
        mechXbox.rightTrigger().whileTrue(new SetShootingConditions(drivetrain, m_turret, m_shooter, false, false));
        mechXbox.y().whileTrue(m_intakePositioner.IntakePosUp(IntakePositionerConstants.kIntakePositionerSpeed));
        mechXbox.a().whileTrue(m_intakePositioner.IntakePosDown(IntakePositionerConstants.kIntakePositionerSpeed));
        //mechXbox.a().whileTrue(new SetIntakeAngleDown(m_intakePositioner, false));
        mechXbox.povUp().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint1, false));
        mechXbox.povRight().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint2, false));
        //mechXbox.povDown().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint3, false));
        mechXbox.povDown().onTrue(m_intakePositioner.IntakePosDown(IntakePositionerConstants.kLowSpeed));
/* 
        mechXbox.x().whileTrue(m_turret.GoToPos(-40));
        mechXbox.y().whileTrue(m_turret.GoToPos(0));
        mechXbox.b().whileTrue(m_turret.GoToPos(40));
*/


/*
        driverXbox.povUp().whileTrue(m_climber.ClimbForward());
        driverXbox.povDown().whileTrue(m_climber.ClimbBackward());
        driverXbox.x().onTrue(m_climber.EngageServo());
        driverXbox.b().onTrue(m_climber.DisengageServo());
        mechXbox.rightTrigger().whileTrue(new SetShootingConditions(drivetrain, m_turret, m_shooter, true, false).andThen(Commands.parallel(new SetShootingConditions(drivetrain, m_turret, m_shooter, false, false) , m_hopper_kicker.Intake())));
        mechXbox.leftTrigger().whileTrue(m_intakeWheel.Intake());
        mechXbox.povUp().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint1, false));
        mechXbox.povDown().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint2, false));
        mechXbox.povRight().onTrue(new SetIntakeAngle_MAXMotion(m_intakePositioner, IntakePositionerConstants.kSetpoint3, false));
        mechXbox.x().whileTrue(m_climber.ClimbToSetpoint(ClimberConstants.kSetpoint1));
        mechXbox.y().whileTrue(m_climber.ClimbToSetpoint(ClimberConstants.kSetpoint2));
        mechXbox.b().whileTrue(m_climber.ClimbToSetpoint(ClimberConstants.kSetpoint3));
        mechXbox.a().onTrue(m_climber.SetServoEngagement());
         */


    }

     PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    public Command generateDriveToTargetCommand(int positionNumber) {
    /** You must 'defer' the command in order for it to work properly. Commands are 'static' in that they
     * initialize everything on code startup. In this case, the logic below will always use the pose kZero, because
     * the code is processed on boot. To avoid this, we use 'Commands.defer()' to have the command be generated when
     * we call it from the button press in order to process the logic properly.
     *
     * For a historical perspective, Command-based programming does this in order to 're-use' commands to avoid the
     * memory/CPU overhead costs that would be incurred if commands were constantly generated, which can have an
     * impact on the code's performance due to the roboRIO's specs. So long as you don't do this too much, using
     * 'Commands.defer()' shouldn't cause issues.
     *
     */
    return Commands.defer(()-> {
        var allianceColor = DriverStation.getAlliance();
        Pose2d targetPose = Pose2d.kZero;

        if (allianceColor.isPresent()) {
            //blue alliance targets
            if (allianceColor.get() == Alliance.Blue) {
                targetPose = new Pose2d(3.9, 5.4, Rotation2d.fromDegrees(-88.9));
            } else {
                targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
            }
        }

        // AutoBuilder.pathfindToPose() returns a command, which we use to follow the path
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
    },
    // Use 'Set.of()' to set the Command's Subsystem requirements for a deferred command
    Set.of(drivetrain)).withName("DriveToPose"); // Name the command to help with debugging
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero));
        //return new PathPlannerAuto("Example Auto");
        return autoChooser.getSelected();
        
        //THIS WOULD RESET THE FIELD ORIENTED PERSPECTIVE IN AUTO
        //drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero));
    }
}
