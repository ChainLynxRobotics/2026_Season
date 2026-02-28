// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.*;
import static frc.robot.utils.PointingUtil.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Serializer.SerializerSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterLUT;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.PointingUtil;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;

@Logged
public class RobotContainer {

  private double MaxSpeed =
      1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driveController = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Vision vision = new Vision(drivetrain::passVisionPose, drivetrain::getSimPose);

  private final IntakeSubsystem intake =
      new IntakeSubsystem(new TalonFX(15, kCanBusRio), new TalonFX(16, kCanBusBlinky));

  private final IndexerSubsystem indexer = new IndexerSubsystem(new TalonFX(17, kCanBusBlinky));

  private final SerializerSubsystem serializer =
      new SerializerSubsystem(new TalonFX(18, kCanBusBlinky));

  private SendableChooser<Command> autoChooser;

  @Logged(name = "Shooter")
  public final Shooter shooter =
      new Shooter(
          () -> drivetrain.getState().Pose,
          drivetrain::getSimPose,
          () -> drivetrain.getState().Speeds,
          new TalonFX(ShooterConstants.kFlywheelCANId, kCanBusBlinky),
          new TalonFX(ShooterConstants.kHoodCANId, kCanBusBlinky));

  public RobotContainer() {
    NamedCommands.registerCommand("goToHub", autoAimShooter());
    NamedCommands.registerCommand("Climb", new PrintCommand("Implement it plz"));
    NamedCommands.registerCommand("Shoot Balls", (indexer.spin10V().andThen(serializer.spin())));
    NamedCommands.registerCommand("Stop Shoot", indexer.stopSpin().andThen(serializer.stopSpin()));
    NamedCommands.registerCommand("Scoop", intake.spin5V());
    NamedCommands.registerCommand("Stop Scoop", intake.stopSpin());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    shooter.setDefaultCommand(shooter.runShooterControl());
    intake.setDefaultCommand(intake.runIntakeControl());
    configureBindings();

    // if (Robot.isSimulation()) SimulatedArena.getInstance().resetFieldForAuto();
  }

  public Pose3d[] getGamePieces() {
    if (RobotBase.isReal()) return new Pose3d[0];
    return SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically

        drivetrain.applyRequest(
            () -> {
              if (driveController.b().getAsBoolean()) {
                return drive
                    .withVelocityX(
                        -driveController.getLeftY()
                            * MaxSpeed
                            / kSlowMoveRate) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driveController.getLeftX()
                            * MaxSpeed
                            / kSlowMoveRate) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate);
              } // Drive counterclockwise with negative X (left)
              else {
                return drive
                    .withVelocityX(
                        -driveController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate);
              } // Drive counterclockwise with negative X (left)
            }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

    driveController
        .rightBumper()
        .onTrue(indexer.spin().andThen(serializer.spin()))
        .onFalse(indexer.stopSpin().andThen(serializer.stopSpin()));

    driveController.y().onTrue(indexer.spin()).onFalse(indexer.stopSpin());

    // Reset the field-centric heading on left bumper press.
    driveController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    // driveController.povRight().whileTrue(intake.setHeight()).onFalse(intake.setHeightToZero());

    driveController.leftBumper().toggleOnTrue(autoAimShooter());

    drivetrain.registerTelemetry(logger::telemeterize);

    driveController
        .x()
        .toggleOnTrue(intake.spin5V())
        .toggleOnFalse(intake.stopSpin()); // spin intake
    driveController.povUp().onTrue(shooter.zeroHood().ignoringDisable(true)); // zero hud

    driveController.povLeft().onTrue(runOnce(() -> intake.zeroHeight()).ignoringDisable(true));

    if (RobotBase.isReal()) return;

    driveController.povLeft().onTrue(Commands.runOnce(() -> shooter.shootSimulatedProjectile()));

    driveController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  var pieces = SimulatedArena.getInstance().getGamePiecesByType("Fuel");
                  for (GamePiece fuel : pieces) {

                    SimulatedArena.getInstance().removePiece(fuel);
                  }
                }));

    // driveController
    //     .y()
    //     .onTrue(shooter.setFlywheelVelocity(RotationsPerSecond.of(60)))
    //     .onFalse(shooter.setFlywheelVelocity(RotationsPerSecond.of(0)));
    // driveJoystick
    //     .y()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 // on y button press rotate robot to angle from getAngleToHub()
    //                 new SwerveRequest.FieldCentricFacingAngle()
    //                     .withTargetDirection(getAngleToHub())
    //                     .withHeadingPID(7, 0, 0)
    //                     // ^This pid is vibes for now fyi
    //                     .withVelocityX(
    //                         -driveJoystick.getLeftY()
    //                             * MaxSpeed) // Drive forward with negative Y (forward)
    //                     .withVelocityY(
    //                         -driveJoystick.getLeftX()
    //                             * MaxSpeed))); // Drive left with negative X (left)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // driveJoystick
    //     .back()
    //     .and(driveJoystick.y())
    //     .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driveJoystick
    //     .back()
    //     .and(driveJoystick.x())
    //     .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // driveJoystick
    //     .start()
    //     .and(driveJoystick.y())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driveJoystick
    //     .start()
    //     .and(driveJoystick.x())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  private SwerveRequest.FieldCentricFacingAngle shooterAming =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(6, 1, 0)
          .withDeadband(MaxSpeed * 0.1);

  public Command autoAimShooter() {
    return drivetrain.applyRequest(
        () ->
            shooterAming
                .withTargetDirection(
                    getAngleToTargetTOF()
                        .minus(
                            getAlliance().equals(DriverStation.Alliance.Red)
                                ? new Rotation2d(Degrees.of(180))
                                : new Rotation2d())) // We want our rotation to not be field centric
                // but we do want our driving to be so we
                // manually flip the rotation
                // .withTargetRateFeedforward(getTOFRotationalVelocityToHub())
                .withVelocityX(-driveController.getLeftY() * MaxSpeed)
                .withVelocityY(-driveController.getLeftX() * MaxSpeed));
  }

  public Command goToHub(Supplier<ChassisSpeeds> Speed) {
    return run(() -> CommandSwerveDrivetrain.pathplannerPointAtHub(getAngleToHub()))
        .finallyDo(() -> CommandSwerveDrivetrain.pathplannerClearOverride());
  }

  public Rotation2d hubTrackingError() {
    return getAngleToHub().minus(drivetrain.getPose().getRotation());
  }

  public Rotation2d getAngleToHub() {
    return PointingUtil.getAngleToHub(drivetrain.getPose());
  }

  public Rotation2d getAngleToHubTOF() {
    return PointingUtil.getAngleToHubTOF(
        drivetrain.getPose(),
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation()));
  }

  public Rotation2d getAngleToTargetTOF() {
    return PointingUtil.getAngleToPoseTOF(
        drivetrain.getPose(),
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation()),
        getShootingTarget(drivetrain.getPose()));
  }

  public AngularVelocity getTOFRotationalVelocityToHub() {
    var drivetrainFieldRelitiveSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation());
    return PointingUtil.getTOFRotationalVelocityToHub(
        drivetrain.getPose(), drivetrainFieldRelitiveSpeeds);
  }

  public AngularVelocity getRotationalVelocityToHub() {
    var drivetrainFieldRelitiveSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation());
    return PointingUtil.getRotationalVelocityToHub(
        drivetrain.getPose(), drivetrainFieldRelitiveSpeeds);
  }

  Pose2d lastTOFPose = new Pose2d();

  public Pose2d getTOFPose() {
    var setpoint =
        ShooterLUT.generateShootOnTheMoveSetpoint(
            drivetrain.getPose(),
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getPose().getRotation()),
            getShootingTarget(drivetrain.getPose()));
    if (setpoint.isEmpty()) {
      return lastTOFPose;
    }
    var pose = setpoint.get().iteratedPose();
    lastTOFPose = new Pose2d(pose.getX(), pose.getY(), setpoint.get().robotRotation());
    return lastTOFPose;
  }

  public Pose2d getTarget() {
    return getShootingTarget(drivetrain.getPose());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("shoot");

    // Simple drive forward auton
    /*
    /*
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain
            .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));

    */
  }
}
