// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.utils.PointingUtil.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Serializer.SerializerSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterLUT;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.utils.PointingUtil;
import frc.robot.utils.TunableNumber;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {

  private boolean doDriving;
  private boolean doTrenchAlign;

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
  @Logged private final LedSubsystem ledSubsystem = new LedSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController testingController = new CommandXboxController(1);
  @Logged public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged
  public final Vision vision = new Vision(drivetrain::passVisionPose, drivetrain::getSimPose);

  @Logged
  private final IntakeSubsystem intake =
      new IntakeSubsystem(new TalonFX(15, kCanBusRio), new TalonFX(16, kCanBusRio));

  @Logged
  private final IndexerSubsystem indexer = new IndexerSubsystem(new TalonFX(17, kCanBusBlinky));

  @Logged
  private final SerializerSubsystem serializer =
      new SerializerSubsystem(new TalonFX(18, kCanBusBlinky));

  @Logged private final Climber climber = new Climber(new TalonFX(kClimberId, kCanBusBlinky));

  private SendableChooser<Command> autoChooser;

  private TunableNumber tunableHeadingP = new TunableNumber("tunableHeadingP", 8);
  private TunableNumber tunableHeadingI = new TunableNumber("tunableHeadingI", 0);
  private TunableNumber tunableHeadingD = new TunableNumber("tunableHeadingD", 1);
  private TunableNumber tunableHeadingFFMult = new TunableNumber("tunableHeadingFFMult", 1.0);

  @Logged(name = "Shooter")
  public final Shooter shooter =
      new Shooter(
          () -> drivetrain.getState().Pose,
          drivetrain::getSimPose,
          () -> drivetrain.getState().Speeds,
          new TalonFX(ShooterConstants.kFlywheelCANId, kCanBusBlinky),
          new TalonFX(ShooterConstants.kHoodCANId, kCanBusBlinky),
          () -> (vision.getVisionPose() != null),
          () -> driveController);

  public RobotContainer() {
    NamedCommands.registerCommand(
        "goToHub", new PrintCommand("use pathplanner point at not commands"));
    NamedCommands.registerCommand("Climb", new PrintCommand("climber disabled"));
    // climber.run(() -> climber.setStateSetpoint(ClimberState.BOTTOM)));
    NamedCommands.registerCommand("shootBalls", (indexer.spin().alongWith(serializer.spin())));
    NamedCommands.registerCommand(
        "stopShoot", (indexer.stopSpin().alongWith(serializer.stopSpin())));
    NamedCommands.registerCommand("Scoop", new PrintCommand(""));
    NamedCommands.registerCommand("Stop Scoop", new PrintCommand(""));
    this.doDriving = true;
    this.doTrenchAlign = false;
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    shooter.setDefaultCommand(shooter.runShooterControl());
    intake.setDefaultCommand(intake.runIntakeControl());
    if (false) {
      new Trigger(
              () ->
                  isPoseInSquare(
                      drivetrain.getState().Pose,
                      Constants.getTrenchCorners(getClosestTrench(drivetrain.getState().Pose))[0],
                      Constants.getTrenchCorners(getClosestTrench(drivetrain.getState().Pose))[1]))
          .onTrue(runOnce(() -> climber.setStateSetpoint(ClimberState.BOTTOM)));
    }
    if (true) {
      new Trigger(shooter::hasAStuckBall)
          .onTrue(
              shooter
                  .setFlywheelVelocity(RotationsPerSecond.of(-60))
                  .withName("Unjam ball")
                  .andThen(waitSeconds(1)));

      configureBindings();
    }

    if (Robot.isSimulation()) SimulatedArena.getInstance().resetFieldForAuto();

    new Trigger(DriverStation::isAutonomousEnabled).whileTrue(ledSubsystem.autonomousPattern());
    new Trigger(DriverStation::isTeleopEnabled)
        .onTrue(runOnce(() -> ledSubsystem.calculateShifts()));
    new Trigger(this::isScoringPhaseSoon).whileTrue(ledSubsystem.hubShiftPattern());
    new Trigger(() -> DriverStation.getMatchTime() > 135)
        .whileTrue(ledSubsystem.activePhasePattern());
    new Trigger(() -> DriverStation.getMatchTime() < 30 && DriverStation.isTeleopEnabled())
        .whileTrue(ledSubsystem.endGamePattern());

    new Trigger(ledSubsystem::isPhaseA).whileTrue(ledSubsystem.phaseAPattern());
    new Trigger(ledSubsystem::isPhaseB).whileTrue(ledSubsystem.phaseBPattern());
  }

  public Pose3d[] getGamePieces() {
    if (RobotBase.isReal()) return new Pose3d[0];
    return SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    if (doDriving) {
      drivetrain.setDefaultCommand(
          //     // Drivetrain will execute this command periodically

          drivetrain.applyRequest(
              () -> {
                if (driveController.rightBumper().getAsBoolean()) {
                  return drive
                      .withVelocityX(
                          -driveController.getLeftY()
                              * MaxSpeed
                              / kSlowMoveRate) // Drive forward with negative Y (forward)
                      .withVelocityY(
                          -driveController.getLeftX()
                              * MaxSpeed
                              / kSlowMoveRate) // Drive left with negative X (left)
                      .withRotationalRate(
                          -driveController.getRightX() * MaxAngularRate / kSlowMoveRate)
                      .withDeadband(0.1 * MaxSpeed / kSlowMoveRate)
                      .withRotationalDeadband(0.1 * MaxAngularRate / kSlowMoveRate);
                } // Drive counterclockwise with negative X (left)
                if (doTrenchAlign && driveController.leftTrigger().getAsBoolean()) {
                  return trenchAlign
                      .withTargetDirection(handleTrenchAlignment().get())
                      .withVelocityX(calculateTrenchAlignSpeeds().vxMetersPerSecond)
                      .withVelocityY(calculateTrenchAlignSpeeds().vyMetersPerSecond);
                } else {
                  return drive
                      .withVelocityX(
                          -driveController.getLeftY()
                              * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(
                          -driveController.getLeftX() * MaxSpeed) // Drive left with negative X
                      .withRotationalRate(-driveController.getRightX() * MaxAngularRate);
                } // Drive counterclockwise with negative X (left)
              }));
    }

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driveController
        .rightTrigger()
        .onTrue(
            waitSeconds(0.75)
                .andThen(
                    indexer
                        .spin()
                        .alongWith(
                            serializer
                                .spin()
                                .alongWith(
                                    repeatingSequence(
                                        waitSeconds(2)
                                            .andThen(runOnce(() -> indexer.swapIndexerDir()))
                                            .andThen(
                                                waitSeconds(0.25)
                                                    .andThen(
                                                        runOnce(
                                                            () -> indexer.swapIndexerDir()))))))))
        .onFalse(indexer.stopSpin().alongWith(serializer.stopSpin()));

    driveController
        .rightTrigger()
        .whileTrue(
            autoAimShooter()
                .alongWith(
                    run(
                        () -> {
                          if (hubTrackingError().getDegrees() < 3
                              && isWithinTolerance(
                                  shooter.getHoodPosition(),
                                  Degrees.of(shooter.getHoodClosedLoopReference()),
                                  Degrees.of(3))) {
                            indexer.spin().alongWith(serializer.spin());
                          } else {
                            indexer.stopSpin().alongWith(serializer.stopSpin());
                          }
                        })));

    /*.onlyWhile(
    () ->
        (hubTrackingError().getDegrees() > 3
            && !isWithinTolerance(
                shooter.getHoodPosition(),
                Degrees.of(shooter.getHoodClosedLoopReference()),
                Degrees.of(1)))));*/

    driveController.x().onTrue(runOnce(() -> intake.swapIntake()));

    driveController.povLeft().onTrue((runOnce(() -> intake.swapIntakeHeight())));

    driveController.y().whileTrue(intake.raiseIntake());

    driveController.b().onTrue((runOnce(() -> indexer.swapIndexerDir())));

    // driveController.povRight().onTrue(runOnce(() ->
    // climber.zeroClimber()).ignoringDisable(true));

    driveController.povRight().whileTrue(intake.raiseIntakeOcilate());

    driveController
        .a()
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    driveController.leftBumper().toggleOnTrue(autoAimShooter());

    driveController.povUp().onTrue(runOnce(() -> climber.setStateSetpoint(ClimberState.TOP)));

    driveController.povDown().onTrue(runOnce(() -> climber.setStateSetpoint(ClimberState.BOTTOM)));

    drivetrain.registerTelemetry(logger::telemeterize);

    if (RobotBase.isReal()) return;
  }

  private SwerveRequest.FieldCentricFacingAngle shooterAming =
      new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.1 / kSlowMoveRate);

  private SwerveRequest.FieldCentricFacingAngle trenchAlign =
      new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(8, 0, 0);

  public Command autoAimShooter() {
    return drivetrain.applyRequest(
        () ->
            shooterAming
                .withTargetDirection(
                    getAngleToTargetTOF()
                        .minus(
                            getAlliance().equals(DriverStation.Alliance.Red)
                                ? new Rotation2d(Degrees.of(180))
                                : new Rotation2d())) // We want our rotation to not be fieldcentric
                // but we do want our driving to be so we
                // manually flip the rotation
                .withTargetRateFeedforward(
                    getTOFRotationalVelocityToTarget(getShootingTarget(drivetrain.getPose())))
                .withHeadingPID(tunableHeadingP.get(), tunableHeadingI.get(), tunableHeadingD.get())
                .withVelocityX(-driveController.getLeftY() * MaxSpeed / (kSlowMoveRate))
                .withVelocityY(-driveController.getLeftX() * MaxSpeed / (kSlowMoveRate)));
  }

  public Optional<Rotation2d> handleTrenchAlignment() {
    if (-90 < drivetrain.getState().Pose.getRotation().getDegrees()
        && drivetrain.getState().Pose.getRotation().getDegrees() < 90) {
      return Optional.of(new Rotation2d(Degrees.of(0)));
    }
    if (90 < drivetrain.getState().Pose.getRotation().getDegrees()
        && drivetrain.getState().Pose.getRotation().getDegrees() < -90) {
      return Optional.of(new Rotation2d(Degrees.of(180)));
    }
    return Optional.empty();
  }

  @Logged
  public ChassisSpeeds calculateTrenchAlignSpeeds() {
    double kP = 4;
    double centerOfTrenchY = getTrenchCenter(getClosestTrench(drivetrain.getPose())).getY();
    double ySpeed = kP * (centerOfTrenchY - drivetrain.getState().Pose.getY());
    double xSpeed =
        ((centerOfTrenchY - drivetrain.getState().Pose.getY()) > 1.5)
            ? -drivetrain.getState().Speeds.vxMetersPerSecond
            : -driveController.getLeftY() * MaxSpeed / 2;
    return new ChassisSpeeds(xSpeed, ySpeed, 0);
  }

  public Command goToHub(Supplier<ChassisSpeeds> Speed) {
    return run(() -> CommandSwerveDrivetrain.pathplannerPointAtHub(getAngleToHub()))
        .finallyDo(() -> CommandSwerveDrivetrain.pathplannerClearOverride());
  }

  @Logged
  public Rotation2d hubTrackingError() {
    return getAngleToHub().minus(drivetrain.getPose().getRotation());
  }

  @Logged
  public Rotation2d getAngleToHub() {
    return PointingUtil.getAngleToHub(drivetrain.getPose());
  }

  @Logged
  public Rotation2d getAngleToHubTOF() {
    return PointingUtil.getAngleToHubTOF(
        drivetrain.getPose(),
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation()));
  }

  @Logged
  public Rotation2d getAngleToTargetTOF() {
    return PointingUtil.getAngleToPoseTOF(
        drivetrain.getPose(),
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation()),
        getShootingTarget(drivetrain.getPose()));
  }

  public AngularVelocity getTOFRotationalVelocityToTarget(Pose2d target) {
    var drivetrainFieldRelitiveSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation());
    return PointingUtil.getTOFRotationalVelocityToTarget(
        drivetrain.getPose(), drivetrainFieldRelitiveSpeeds, target);
  }

  @Logged
  public AngularVelocity getTOFRotationalVelocityReal() {
    return getTOFRotationalVelocityToTarget(PointingUtil.getShootingTarget(drivetrain.getPose()));
  }

  @Logged
  public AngularVelocity getRotationalVelocityToHub() {
    var drivetrainFieldRelitiveSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getPose().getRotation());
    return PointingUtil.getRotationalVelocityToHub(
        drivetrain.getPose(), drivetrainFieldRelitiveSpeeds);
  }

  Pose2d lastTOFPose = new Pose2d();

  @Logged
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

  @Logged
  public Pose2d getTarget() {
    return getShootingTarget(drivetrain.getPose());
  }

  @Logged
  public Pose2d getTrenchCorner1() {
    return getTrenchCornersVelocity(
        getClosestTrench(drivetrain.getState().Pose),
        drivetrain.getState().Speeds,
        drivetrain.getState().Pose)[0];
  }

  @Logged
  public Pose2d getTrenchCorner2() {
    return getTrenchCornersVelocity(
        getClosestTrench(drivetrain.getState().Pose),
        drivetrain.getState().Speeds,
        drivetrain.getState().Pose)[1];
  }

  @Logged
  public boolean isScoringPhaseSoon() {
    double time = Timer.getMatchTime();
    if ((time > 130 && time < 135)
        || (time > 105 && time < 110)
        || (time > 80 && time < 85)
        || (time > 55 && time < 60)
        || (time > 30 && time < 35)) {
      return true;
    } else {
      return false;
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    // Simple drive forward auton

    /*final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain
            .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));*/

  }
}
