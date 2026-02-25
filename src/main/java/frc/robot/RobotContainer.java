// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.Constants.getAlliance;
import static frc.robot.Constants.getHubLocation2d;
import static frc.robot.Constants.kCanBusBlinky;
import static frc.robot.Constants.kCanBusRio;
import static frc.robot.subsystems.Shooter.ShooterConstants.kShooterLocation;

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
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

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
      new IntakeSubsystem(new TalonFX(15, kCanBusRio), new TalonFX(16, kCanBusRio));

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
    NamedCommands.registerCommand("goToHub", goToHub());
    NamedCommands.registerCommand("Climb", new PrintCommand("Implement it plz"));
    NamedCommands.registerCommand("Shoot Balls", (indexer.spin10V().andThen(serializer.spin())));
    NamedCommands.registerCommand("Stop Shoot", indexer.stopSpin().andThen(serializer.stopSpin()));
    NamedCommands.registerCommand("Scoop", intake.spin5V());
    NamedCommands.registerCommand("Stop Scoop", intake.stopSpin());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    shooter.setDefaultCommand(shooter.runShooterControl());
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
            () ->
                drive
                    .withVelocityX(
                        -driveController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driveController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

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
        .onTrue(indexer.spin10V().andThen(serializer.spin()))
        .onFalse(indexer.stopSpin().andThen(serializer.stopSpin()));
    // driveController
    //     .leftTrigger()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               var pieces = SimulatedArena.getInstance().getGamePiecesByType("Fuel");
    //               for (GamePiece fuel : pieces) {

    //                 SimulatedArena.getInstance().removePiece(fuel);
    //               }
    //             }));

    driveController.y().onTrue(indexer.spin10V()).onFalse(indexer.stopSpin());

    driveController.a().toggleOnTrue(goToHub());
    driveController
        .povRight()
        .onTrue(
            shooter
                .setFlywheelVelocity(RotationsPerSecond.of(0))
                .andThen(shooter.setHoodAngle(Degrees.of(5))));

    // Reset the field-centric heading on left bumper press.
    driveController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
    // driveController.povLeft().onTrue(Commands.runOnce(() -> shooter.shootSimulatedProjectile()));

    driveController.x().toggleOnTrue(intake.spin5V()).toggleOnFalse(intake.stopSpin());
    driveController.povUp().onTrue(shooter.zeroHood().ignoringDisable(true));

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

  private SwerveRequest.FieldCentricFacingAngle pointAtHub =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(6, 1, 0)
          .withDeadband(MaxSpeed * 0.1);

  public Command goToHub() {
    return drivetrain.applyRequest(
        () ->
            pointAtHub
                .withTargetDirection(
                    getAngleToHubTOF()
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
    return getAngleToHub(drivetrain.getPose());
  }

  public static Rotation2d getAngleToHub(Pose2d robotPose) {
    return Shooter.getShooterPose(robotPose)
        .getTranslation()
        .minus(getHubLocation2d().getTranslation())
        .getAngle()
        .plus(kShooterLocation.getRotation());
  }

  Rotation2d lastRotation = new Rotation2d();

  public Rotation2d getAngleToHubTOF(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    var setpoint = ShooterLUT.generateShootOnTheMoveSetpoint(robotPose, robotSpeeds);
    if (setpoint.isEmpty()) {
      return lastRotation;
    }
    lastRotation = setpoint.get().robotRotation();
    return lastRotation;
  }

  public Rotation2d getAngleToHubTOF() {
    return getAngleToHubTOF(
        drivetrain.getPose(),
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation()));
  }

  public AngularVelocity getTOFRotationalVelocityToHub() {
    var dt = 0.01;
    var drivetrainFieldRelitiveSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation());
    var poseInDt =
        new Pose2d(
            drivetrain.getPose().getX() + drivetrainFieldRelitiveSpeeds.vxMetersPerSecond * dt,
            drivetrain.getPose().getY() + drivetrainFieldRelitiveSpeeds.vyMetersPerSecond * dt,
            new Rotation2d(
                drivetrain.getPose().getRotation().getRadians()
                    + drivetrainFieldRelitiveSpeeds.omegaRadiansPerSecond * dt));
    return RotationsPerSecond.of(
        getAngleToHubTOF(poseInDt, drivetrainFieldRelitiveSpeeds)
                .getMeasure()
                .minus(
                    getAngleToHubTOF(drivetrain.getPose(), drivetrainFieldRelitiveSpeeds)
                        .getMeasure())
                .in(Rotations)
            / dt);
  }

  public AngularVelocity getRotationalVelocityToHub() {
    var dt = 0.01;
    var drivetrainFieldRelitiveSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation());
    var poseInDt =
        new Pose2d(
            drivetrain.getPose().getX() + drivetrainFieldRelitiveSpeeds.vxMetersPerSecond * dt,
            drivetrain.getPose().getY() + drivetrainFieldRelitiveSpeeds.vyMetersPerSecond * dt,
            new Rotation2d(
                drivetrain.getPose().getRotation().getRadians()
                    + drivetrainFieldRelitiveSpeeds.omegaRadiansPerSecond * dt));
    return RotationsPerSecond.of(
        getAngleToHub(poseInDt).getMeasure().minus(getAngleToHub().getMeasure()).in(Rotations)
            / dt);
  }

  Pose2d lastTOFPose = new Pose2d();

  public Pose2d getTOFPose() {
    var setpoint =
        ShooterLUT.generateShootOnTheMoveSetpoint(
            drivetrain.getPose(),
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getPose().getRotation()));
    if (setpoint.isEmpty()) {
      return lastTOFPose;
    }
    var pose = setpoint.get().iteratedPose();
    lastTOFPose = new Pose2d(pose.getX(), pose.getY(), setpoint.get().robotRotation());
    return lastTOFPose;
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
