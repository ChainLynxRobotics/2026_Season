// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.utils.PointingUtil.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.utils.PointingUtil;
import frc.robot.utils.RobotMath;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.simulation.IntakeSim;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {

  private boolean doDriving;
  private boolean doTrenchAlign;
  private boolean doInstantShoot = true;

  private Timer climbTimer = new Timer();

  private double MaxSpeed =
      1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.15)
          .withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController driveController = new CommandXboxController(0);

  // private final CommandXboxController testingController = new CommandXboxController(1);

  @Logged public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged
  public final Vision vision =
      new Vision(drivetrain::passVisionPose, drivetrain::getSimPose, drivetrain::getPose);

  @Logged
  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          new TalonFX(15, kCanBusRio), new TalonFX(16, kCanBusRio), () -> getInstantShootActive());

  @Logged
  private final IndexerSubsystem indexer = new IndexerSubsystem(new TalonFX(17, kCanBusBlinky));

  @Logged
  private final SerializerSubsystem serializer =
      new SerializerSubsystem(new TalonFX(18, kCanBusBlinky));

  @Logged
  private final Climber climber =
      new Climber(new TalonFX(ClimberConstants.kClimberId, kCanBusBlinky));

  // @Logged private final LedSubsystem ledSubsystem = new LedSubsystem();

  private SendableChooser<Command> autoChooser;

  private TunableNumber tunableHeadingP = new TunableNumber("tunableHeadingP", 10);
  private TunableNumber tunableHeadingI = new TunableNumber("tunableHeadingI", 0);
  private TunableNumber tunableHeadingD = new TunableNumber("tunableHeadingD", 0);
  private TunableNumber tunableHeadingFFMult = new TunableNumber("tunableHeadingFFMult", 1.0);

  private double[] lastDriveInput = {0.0, 0.0};

  @Logged(name = "Shooter")
  public final Shooter shooter =
      new Shooter(
          () -> drivetrain.getState().Pose,
          drivetrain::getSimPose,
          () -> drivetrain.getState().Speeds,
          new TalonFX(ShooterConstants.kFlywheelCANId, kCanBusBlinky),
          new TalonFX(ShooterConstants.kFlywheelFollowerCANId, kCanBusBlinky),
          new TalonFX(ShooterConstants.kHoodCANId, kCanBusBlinky),
          () -> (vision.getVisionPose() != null),
          () -> driveController);

  private final Telemetry logger =
      new Telemetry(MaxSpeed, shooter::getHoodPose, indexer::getIndexerPose, intake::getHeightPose);

  public IntakeSim intakeSim;

  private InterpolatingDoubleTreeMap tofMap = ShooterLUT.generateTOFMap();

  public RobotContainer() {
    NamedCommands.registerCommand(
        "goToHub", new PrintCommand("use pathplanner point at not commands"));
    NamedCommands.registerCommand("Climb", climber.climberDeadReckoning(false));
    NamedCommands.registerCommand("shootBalls", (indexer.spin().alongWith(serializer.spin())));
    NamedCommands.registerCommand(
        "stopShoot",
        (indexer
            .stopSpin()
            .alongWith(serializer.stopSpin())
            .alongWith(
                runOnce(
                    () -> {
                      CommandScheduler.getInstance().cancel(intake.raiseIntakeOscillate());
                    }))));
    NamedCommands.registerCommand("Scoop", new PrintCommand(""));
    NamedCommands.registerCommand("Stop Scoop", new PrintCommand(""));
    NamedCommands.registerCommand("deployIntake", intake.deployIntake());
    NamedCommands.registerCommand("runShootingCommands", runShootingCommandsSlow());
    NamedCommands.registerCommand("runTowerAlign", runTowerAlign());
    NamedCommands.registerCommand(
        "runShootingCommandsAutoNegX",
        runShootingCommandsSlowAuto(-1.0, 0.0, kSOTMBackLeftPoseSquareBlueAlliance));
    NamedCommands.registerCommand(
        "runShootingCommandsAutoPosXPosY",
        runShootingCommandsSlowAutoNoIntake(0.75, 0.75, kSOTMTopTrenchPoseSquareBlueAlliance));
    NamedCommands.registerCommand(
        "runShootingCommandsAutoNegY",
        runShootingCommandsSlowAuto(0, -1.0, kSOTMAfterDepoPoseSquareBlueAlliance));
    this.doDriving = true;
    this.doTrenchAlign = true;
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    shooter.setDefaultCommand(shooter.runShooterControl());
    intake.setDefaultCommand(intake.runIntakeControl());

    if (RobotBase.isSimulation()) {
      intakeSim =
          new IntakeSim(drivetrain.getSwerveDriveSimulation(), shooter::shootSimulatedProjectile);
    }

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
    }

    /*new Trigger(DriverStation::isTeleopEnabled)
        .onTrue(runOnce(() -> ledSubsystem.calculateShifts()));
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.AUTO)
        .whileTrue(ledSubsystem.autonomousPattern());
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.SHIFTCHANGE)
        .whileTrue(ledSubsystem.hubShiftPattern());
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.INACTIVE)
        .whileTrue(ledSubsystem.defendingPhasePattern());
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.ACTIVE)
        .whileTrue(ledSubsystem.activePhasePattern());
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.ENDGAME)
        .whileTrue(ledSubsystem.endGamePattern());
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.SHOOTING)
        .whileTrue(ledSubsystem.shootPattern());
    new Trigger(() -> ledSubsystem.getRobotState() == RobotState.DISABLED)
        .whileTrue(ledSubsystem.teamColorPattern());*/

    configureBindings();
    if (kPlayFightSongOnStartup) {
      var orchestra = new Orchestra();
      drivetrain.addMotorsToOrchestra(orchestra);
      shooter.addMotorsToOrchestra(orchestra);
      intake.addMotorsToOrchestra(orchestra);
      serializer.addMotorsToOrchestra(orchestra);
      indexer.addMotorsToOrchestra(orchestra);
      climber.addMotorsToOrchestra(orchestra);

      // orchestra.loadMusic("across-the-field.chrp");
      orchestra.loadMusic("smb.chrp");
      orchestra.play();
    }
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
                if (doTrenchAlign
                    && driveController.leftTrigger().getAsBoolean()
                    && isPoseInSquare(
                        drivetrain.getState().Pose,
                        getTrenchCornersVelocity(
                            getClosestTrench(drivetrain.getState().Pose),
                            drivetrain.getState().Speeds,
                            drivetrain.getState().Pose)[0],
                        getTrenchCornersVelocity(
                            getClosestTrench(drivetrain.getState().Pose),
                            drivetrain.getState().Speeds,
                            drivetrain.getState().Pose)[1])
                    && handleTrenchAlignmentAngle().isPresent()) {
                  return trenchAlign
                      .withTargetDirection(handleTrenchAlignmentAngle().get())
                      .withVelocityX(calculateTrenchAlignSpeeds().vxMetersPerSecond)
                      .withVelocityY(calculateTrenchAlignSpeeds().vyMetersPerSecond);
                }
                if (driveController.leftBumper().getAsBoolean()
                    || isPoseInSquare(
                        drivetrain.getPose(),
                        getTrenchCornersVelocity(
                            getClosestTrench(drivetrain.getPose()),
                            drivetrain.getState().Speeds,
                            drivetrain.getPose())[0],
                        getTrenchCornersVelocity(
                            getClosestTrench(drivetrain.getPose()),
                            drivetrain.getState().Speeds,
                            drivetrain.getPose())[1])) {
                  if (shouldIndex()
                      && Math.abs(driveController.getLeftX()) < (0.15 * MaxSpeed / kSlowMoveRate)
                      && Math.abs(driveController.getLeftY()) < (0.15 * MaxSpeed / kSlowMoveRate)
                      && Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) < 0.1
                      && kShouldUseXBrakeWhenShooting) {
                    return new SwerveDriveBrake();
                  }
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
                      .withDeadband(0.15 * MaxSpeed / kSlowMoveRate)
                      .withRotationalDeadband(0.15 * MaxAngularRate / kSlowMoveRate);
                } else {
                  return drive
                      .withVelocityX(
                          -driveController.getLeftY()
                              * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(
                          -driveController.getLeftX() * MaxSpeed) // Drive left with negative X
                      .withRotationalRate(-driveController.getRightX() * MaxAngularRate);
                } // Drive counterclockwise with negative X (left) // Drive counterclockwise with
                // negative X (left)
              }));
    }

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driveController
        .rightTrigger()
        .whileTrue(runShootingCommandsSlow())
        .onTrue(
            runOnce(
                () -> {
                  shooter.isShooting = true;
                  indexer.isReverseIndexer = false;
                  shooter.flywheelSpikeTimer.reset();
                  shooter.flywheelSpikeTimer.start();
                }))
        .onFalse(
            indexer
                .stopSpin()
                .alongWith(serializer.stopSpin())
                .alongWith(
                    runOnce(
                        () -> {
                          shooter.isShooting = false;
                        })));

    driveController
        .rightBumper()
        .whileTrue(runShootingCommandsSlowNoIntake())
        .onTrue(
            runOnce(
                () -> {
                  shooter.isShooting = true;
                  indexer.isReverseIndexer = false;
                  shooter.flywheelSpikeTimer.reset();
                  shooter.flywheelSpikeTimer.start();
                }))
        .onFalse(
            indexer
                .stopSpin()
                .alongWith(serializer.stopSpin())
                .alongWith(
                    runOnce(
                        () -> {
                          shooter.isShooting = false;
                        })));

    driveController.x().onTrue(runOnce(() -> intake.swapIntake()));

    // driveController.povUp().onTrue(intake.deployIntake());

    driveController.povUp().onTrue(runTowerAlign());

    // driveController.povRight().onTrue(runOnce(() ->
    // climber.zeroClimber()).ignoringDisable(true));

    // driveController.povLeft().onTrue(runOnce(() ->
    // climber.setStateSetpoint(ClimberState.REZERO)));

    driveController
        .povDown()
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    // driveController.povUp().onTrue(runOnce(() -> climber.setStateSetpoint(ClimberState.TOP)));

    // driveController.povDown().onTrue(runOnce(() ->
    // climber.setStateSetpoint(ClimberState.BOTTOM)));

    driveController
        .y()
        .whileTrue(climber.climberDeadReckoning(true))
        .onFalse(runOnce(() -> climber.stopMotor()));

    driveController
        .a()
        .whileTrue(climber.climberDeadReckoning(false))
        .onFalse(runOnce(() -> climber.stopMotor()));

    driveController.b().onTrue(runOnce(() -> doInstantShoot = !doInstantShoot));

    drivetrain.registerTelemetry(logger::telemeterize);

    // testingController.b().onTrue(runOnce(() -> vision.resetSTDevData()));

    if (RobotBase.isReal()) return;
  }

  private SwerveRequest.FieldCentricFacingAngle shooterAiming =
      new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.15 / kSlowMoveRate / 2);

  private SwerveRequest.FieldCentricFacingAngle trenchAlign =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(8, 0, 0)
          .withDeadband(MaxSpeed * 0.15);

  private SwerveRequest.FieldCentricFacingAngle climberAlign =
      new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(2, 0, 0);

  private Command runShootingCommands() {

    return parallel(
        autoAimShooterMotionProfile(),
        run(
            () -> {
              if (Math.abs(targetTOFTrackingError().getDegrees()) < 6
                  && isWithinTolerance(
                      shooter.getHoodPosition(),
                      Degrees.of(shooter.getHoodClosedLoopReference()),
                      Degrees.of(1.5))) {
                indexer.spinInternal();
                serializer.spinInternal();
                if (RobotBase.isSimulation()) {
                  intakeSim.shootGamePiece();
                }
              } else {
                indexer.stopSpinInternal();
                serializer.stopSpinInternal();
              }
            }),
        run(
            () -> {
              if (shooter.timeSinceLastBall().in(Seconds) > indexer.getIndexerUnjamTime()
                  && !indexer.isReverseIndexer) {
                indexer.isReverseIndexer = true;
                shooter.flywheelSpikeTimer.reset();
              } else if (shooter.timeSinceLastBall().in(Seconds) > indexer.getIndexerRestartTime()
                  && indexer.isReverseIndexer) {
                indexer.isReverseIndexer = false;
                shooter.flywheelSpikeTimer.reset();
              }
            }),
        intake.raiseIntakeOscillate());
  }

  private Command runShootingCommandsSlow() {

    return parallel(
            runOnce(
                () -> {
                  shooter.isShooting = true;
                }),
            autoAimShooterMotionProfile(),
            run(
                () -> {
                  if (Math.abs(targetTOFTrackingError().getDegrees()) < 6
                      && isWithinTolerance(
                          shooter.getHoodPosition(),
                          Degrees.of(shooter.getHoodClosedLoopReference()),
                          Degrees.of(1.5))) {
                    if (getInstantShootActive()) {
                      indexer.spinInternal();
                      serializer.spinInternal();
                    }
                    if (RobotBase.isSimulation()) {
                      intakeSim.shootGamePiece();
                    }
                  } else {
                    indexer.stopSpinInternal();
                    serializer.stopSpinInternal();
                  }
                }),
            run(
                () -> {
                  if (shooter.timeSinceLastBall().in(Seconds) > indexer.getIndexerSlowTime()
                      && !indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = true;
                    shooter.flywheelSpikeTimer.reset();
                  } else if (shooter.timeSinceLastBall().in(Seconds)
                          > indexer.getIndexerSpeedUpTime()
                      && indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = false;
                    shooter.flywheelSpikeTimer.reset();
                  }
                }),
            intake.raiseIntakeOscillate())
        .finallyDo(
            () -> {
              shooter.isShooting = false;
            });
  }

  private Command runShootingCommandsSlowNoIntake() {

    return parallel(
            runOnce(
                () -> {
                  shooter.isShooting = true;
                }),
            autoAimShooterMotionProfile(),
            run(
                () -> {
                  if (Math.abs(targetTOFTrackingError().getDegrees()) < 6
                      && isWithinTolerance(
                          shooter.getHoodPosition(),
                          Degrees.of(shooter.getHoodClosedLoopReference()),
                          Degrees.of(1.5))
                      && getInstantShootActive()) {
                    indexer.spinInternal();
                    serializer.spinInternal();
                    if (RobotBase.isSimulation()) {
                      intakeSim.shootGamePiece();
                    }
                  } else {
                    indexer.stopSpinInternal();
                    serializer.stopSpinInternal();
                  }
                }),
            run(
                () -> {
                  if (shooter.timeSinceLastBall().in(Seconds) > indexer.getIndexerSlowTime()
                      && !indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = true;
                    shooter.flywheelSpikeTimer.reset();
                  } else if (shooter.timeSinceLastBall().in(Seconds)
                          > indexer.getIndexerSpeedUpTime()
                      && indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = false;
                    shooter.flywheelSpikeTimer.reset();
                  }
                }))
        .finallyDo(
            () -> {
              shooter.isShooting = false;
            });
  }

  @Logged
  public boolean shouldIndex() {
    return (Math.abs(targetTOFTrackingError().getDegrees()) < 6
        && isWithinTolerance(
            shooter.getHoodPosition(),
            Degrees.of(shooter.getHoodClosedLoopReference()),
            Degrees.of(1.5)));
  }

  private TrapezoidProfile turningProfile = new TrapezoidProfile(new Constraints(2, 1));

  public void periodic() {
    double[] curInputs = {
      Math.abs(driveController.getLeftX()), Math.abs(driveController.getLeftY())
    };
    if ((curInputs[0] > 0.15 && lastDriveInput[0] < 0.15 && curInputs[1] < 0.15)
        && (curInputs[1] > 0.15 && lastDriveInput[1] < 0.15 && curInputs[0] < 0.15)) {
      runOnce(
          () -> {
            SOTMOffset = drivetrain.getPose().getRotation().getMeasure();
            profileState =
                new State(0.0, drivetrain.getState().Speeds.omegaRadiansPerSecond / (2 * Math.PI));
            lastState = profileState;
          });
    }
    lastDriveInput = curInputs;

    if (isClimberAligned() && (climbTimer.get() > 5 || !climbTimer.isRunning())) {
      resetClimbingTimer();
    }
  }

  private State profileState = new State();
  private State lastState = new State();
  private Angle SOTMOffset = Degrees.zero();

  @Logged
  public State getProfile() {
    return profileState;
  }

  @Logged
  public State getLastProfileState() {
    return lastState;
  }

  public Pose2d getDrivePoseWithDOT(Rotation2d angle) {
    var drivetrainPose = drivetrain.getPose();
    return new Pose2d(drivetrainPose.getX(), drivetrainPose.getY(), angle);
  }

  @Logged
  public Pose3d[] getAdjustedCameraPositions() {
    Pose3d robotPose = new Pose3d(drivetrain.getPose());
    return new Pose3d[] {
      robotPose.plus(vision.getCameraPositions()[0]), robotPose.plus(vision.getCameraPositions()[1])
    };
  }

  public PathPlannerPath getPathToCorner() {
    return new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.196, 1.077, new Rotation2d(-135)),
            new Pose2d(0.674, 0.941, new Rotation2d(180))),
        new PathConstraints(2.5, 2.5, Math.PI, Math.PI),
        new IdealStartingState(1.5, new Rotation2d(-30)),
        new GoalEndState(0, new Rotation2d(-57)));
  }

  public PathPlannerPath getPathThroughTrench1() {
    return new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(2.573, 1.105, new Rotation2d(-58.2)),
            new Pose2d(4.447, 0.618, new Rotation2d(-3.27)),
            new Pose2d(7.058, 1.667, new Rotation2d(51.885))),
        new PathConstraints(4, 4, 2 * Math.PI, 3 * Math.PI),
        new IdealStartingState(1.5, new Rotation2d(-44.4)),
        new GoalEndState(1.75, new Rotation2d(48.62)));
  }

  public Command autoAimShooterPID() {
    return drivetrain.applyRequest(
        () ->
            shooterAiming
                .withTargetDirection(
                    getAngleToTargetTOF()
                        .minus(
                            getAlliance().equals(DriverStation.Alliance.Red)
                                ? new Rotation2d(Degrees.of(180))
                                : new Rotation2d())) // We want our rotation to not be field centric
                // but we do want our driving to be so we
                // manually flip the rotation
                .withTargetRateFeedforward(0)
                .withHeadingPID(tunableHeadingP.get(), tunableHeadingI.get(), tunableHeadingD.get())
                .withVelocityX(-driveController.getLeftY() * MaxSpeed)
                .withVelocityY(-driveController.getLeftX() * MaxSpeed));
  }

  public Command autoAimShooterMotionProfile() {
    return parallel(
        runOnce(
            () -> {
              SOTMOffset = drivetrain.getPose().getRotation().getMeasure();
              if (getAlliance() == Alliance.Red) {
                SOTMOffset = SOTMOffset.plus(Degrees.of(180));
              }
              profileState =
                  new State(
                      0.0, drivetrain.getState().Speeds.omegaRadiansPerSecond / (2 * Math.PI));
              lastState = profileState;
            }),
        run(
            () -> {
              var turningRateFF =
                  getTOFRotationalVelocityToTarget(getShootingTarget(drivetrain.getPose()))
                      .times(tunableHeadingFFMult.get());
              lastState = profileState;
              profileState =
                  turningProfile.calculate(
                      kDT.in(Second),
                      lastState,
                      new State(
                          PointingUtil.optimiseRotation(
                                  drivetrain
                                      .getPose()
                                      .getRotation()
                                      .minus(new Rotation2d(SOTMOffset)),
                                  getAngleToTargetTOF()
                                      .minus(
                                          getAlliance().equals(DriverStation.Alliance.Red)
                                              ? new Rotation2d(Degrees.of(180))
                                              : new Rotation2d())
                                      .minus(new Rotation2d(SOTMOffset))
                                  // We want our rotation to not be field centric
                                  // but we do want our driving to be so we
                                  // manually flip the rotation
                                  )
                              .getRotations() // We want our rotation to not be field centric
                          // but we do want our driving to be so we
                          // manually flip the rotation
                          ,
                          turningRateFF.in(RotationsPerSecond)));
            }),
        drivetrain.applyRequest(
            () ->
                shooterAiming
                    .withTargetDirection(getAutoAimPositionFromJoystick())
                    .withTargetRateFeedforward(getAutoAimVelocityFromJoystick())
                    .withHeadingPID(
                        tunableHeadingP.get(), tunableHeadingI.get(), tunableHeadingD.get())
                    .withVelocityX(-driveController.getLeftY() * MaxSpeed / kSlowMoveRate / 2)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed / kSlowMoveRate / 2)));
  }

  public Command autoAimShooterMotionProfileAuto(double xSpeed, double ySpeed) {
    return parallel(
        runOnce(
            () -> {
              SOTMOffset = drivetrain.getPose().getRotation().getMeasure();
              profileState =
                  new State(
                      0.0, drivetrain.getState().Speeds.omegaRadiansPerSecond / (2 * Math.PI));
              lastState = profileState;
            }),
        run(
            () -> {
              var turningRateFF =
                  getTOFRotationalVelocityToTarget(getShootingTarget(drivetrain.getPose()))
                      .times(tunableHeadingFFMult.get());
              lastState = profileState;
              profileState =
                  turningProfile.calculate(
                      kDT.in(Second),
                      lastState,
                      new State(
                          PointingUtil.optimiseRotation(
                                  drivetrain
                                      .getPose()
                                      .getRotation()
                                      .minus(new Rotation2d(SOTMOffset)),
                                  getAngleToTargetTOF()
                                      .minus(
                                          getAlliance().equals(DriverStation.Alliance.Red)
                                              ? new Rotation2d(Degrees.of(180))
                                              : new Rotation2d())
                                      .minus(new Rotation2d(SOTMOffset))
                                  // We want our rotation to not be field centric
                                  // but we do want our driving to be so we
                                  // manually flip the rotation
                                  )
                              .getRotations() // We want our rotation to not be field centric
                          // but we do want our driving to be so we
                          // manually flip the rotation
                          ,
                          turningRateFF.in(RotationsPerSecond)));
            }),
        drivetrain.applyRequest(
            () ->
                shooterAiming
                    .withTargetDirection(getAutoAimPositionFromJoystick())
                    .withTargetRateFeedforward(getAutoAimVelocityFromJoystick())
                    .withHeadingPID(
                        tunableHeadingP.get(), tunableHeadingI.get(), tunableHeadingD.get())
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)));
  }

  private Command runShootingCommandsSlowAuto(double xSpeed, double ySpeed, Pose2d[] endPoses) {

    return parallel(
            runOnce(
                () -> {
                  shooter.isShooting = true;
                }),
            autoAimShooterMotionProfileAuto(xSpeed, ySpeed),
            run(
                () -> {
                  if (Math.abs(targetTOFTrackingError().getDegrees()) < 6
                      && isWithinTolerance(
                          shooter.getHoodPosition(),
                          Degrees.of(shooter.getHoodClosedLoopReference()),
                          Degrees.of(1.5))) {
                    if (getInstantShootActive()) {
                      indexer.spinInternal();
                      serializer.spinInternal();
                    }
                    if (RobotBase.isSimulation()) {
                      intakeSim.shootGamePiece();
                    }
                  } else {
                    indexer.stopSpinInternal();
                    serializer.stopSpinInternal();
                  }
                }),
            run(
                () -> {
                  if (shooter.timeSinceLastBall().in(Seconds) > indexer.getIndexerSlowTime()
                      && !indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = true;
                    shooter.flywheelSpikeTimer.reset();
                  } else if (shooter.timeSinceLastBall().in(Seconds)
                          > indexer.getIndexerSpeedUpTime()
                      && indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = false;
                    shooter.flywheelSpikeTimer.reset();
                  }
                }),
            intake.raiseIntakeOscillate())
        .until(() -> RobotMath.isPoseInSquare(drivetrain.getPose(), endPoses[0], endPoses[1]))
        .finallyDo(
            () -> {
              shooter.isShooting = false;
            });
  }

  private Command runShootingCommandsSlowAutoNoIntake(
      double xSpeed, double ySpeed, Pose2d[] endPoses) {

    return parallel(
            runOnce(
                () -> {
                  shooter.isShooting = true;
                }),
            autoAimShooterMotionProfileAuto(xSpeed, ySpeed),
            run(
                () -> {
                  if (Math.abs(targetTOFTrackingError().getDegrees()) < 6
                      && isWithinTolerance(
                          shooter.getHoodPosition(),
                          Degrees.of(shooter.getHoodClosedLoopReference()),
                          Degrees.of(1.5))) {
                    if (getInstantShootActive()) {
                      indexer.spinInternal();
                      serializer.spinInternal();
                    }
                    if (RobotBase.isSimulation()) {
                      intakeSim.shootGamePiece();
                    }
                  } else {
                    indexer.stopSpinInternal();
                    serializer.stopSpinInternal();
                  }
                }),
            run(() -> {
                  if (shooter.timeSinceLastBall().in(Seconds) > indexer.getIndexerSlowTime()
                      && !indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = true;
                    shooter.flywheelSpikeTimer.reset();
                  } else if (shooter.timeSinceLastBall().in(Seconds)
                          > indexer.getIndexerSpeedUpTime()
                      && indexer.isSlowIndexer) {
                    indexer.isSlowIndexer = false;
                    shooter.flywheelSpikeTimer.reset();
                  }
                })
                .until(
                    () -> RobotMath.isPoseInSquare(drivetrain.getPose(), endPoses[0], endPoses[1])))
        .finallyDo(
            () -> {
              shooter.isShooting = false;
            });
  }

  public Optional<Rotation2d> handleTrenchAlignmentAngle() {
    if (isForward()) {
      return Optional.of(new Rotation2d());
    }
    if (!isForward()) {
      return Optional.of(new Rotation2d(Degrees.of(180)));
    } else {
      return Optional.empty();
    }
  }

  @Logged
  public boolean isForward() {
    return (drivetrain.getPose().getRotation().getMeasure().isNear(Degrees.of(0), Degrees.of(90)));
  }

  public Rotation2d getAutoAimPositionFromJoystick() {
    if (Math.abs(driveController.getLeftX()) > 0.15
        || Math.abs(driveController.getLeftY()) > 0.15) {
      return Rotation2d.fromRotations(profileState.position).plus(new Rotation2d(SOTMOffset));
    } else {
      return getAngleToTargetTOF()
          .minus(
              getAlliance().equals(DriverStation.Alliance.Red)
                  ? new Rotation2d(Degrees.of(180))
                  : new Rotation2d());
    }
  }

  public AngularVelocity getAutoAimVelocityFromJoystick() {
    if (Math.abs(driveController.getLeftX()) > 0.15
        || Math.abs(driveController.getLeftY()) > 0.15) {
      return RotationsPerSecond.of(getProfile().velocity);
    } else {
      return RotationsPerSecond.zero();
    }
  }

  public Command endInCorner() {
    return run(() -> {})
        .until(
            () ->
                isPoseInSquare(
                    drivetrain.getPose(),
                    new Pose2d(Meters.of(1.225), Meters.of(6.432), new Rotation2d()),
                    new Pose2d(Meters.of(0), Meters.of(8), new Rotation2d())));
  }

  @Logged
  public ChassisSpeeds calculateTrenchAlignSpeeds() {
    double kP = 6;
    Pose2d centerOfTrench = getTrenchCenter(getClosestTrench(drivetrain.getPose()));
    double ySpeed = kP * (centerOfTrench.getY() - drivetrain.getState().Pose.getY());
    double xSpeed =
        ((Math.abs(centerOfTrench.getY() - drivetrain.getState().Pose.getY())) > 0.25
                && Math.abs(centerOfTrench.getX() - drivetrain.getState().Pose.getX()) < 1.75)
            ? 0
            : -driveController.getLeftY() * MaxSpeed;
    // double ySpeed = -driveController.getLeftX() * MaxSpeed / 2;
    return new ChassisSpeeds(xSpeed, ySpeed, 0);
  }

  public Command runTowerAlign() {
    return drivetrain
        .applyRequest(
            () ->
                climberAlign
                    .withVelocityX(calculateTowerAlignSpeeds().vxMetersPerSecond)
                    .withVelocityY(calculateTowerAlignSpeeds().vyMetersPerSecond)
                    .withTargetDirection(
                        Rotation2d.fromRotations(
                            calculateTowerAlignSpeeds().omegaRadiansPerSecond)))
        .until(() -> isClimberAligned() && climbTimer.hasElapsed(1));
  }

  public void resetClimbingTimer() {
    climbTimer.start();
    climbTimer.reset();
  }

  public boolean isClimberAligned() {
    Pose2d targetPose2d = getAlliance() == Alliance.Red ? kMirrorClimbRed : kMirrorClimbBlue;
    return Math.abs(drivetrain.getPose().minus(targetPose2d).getX()) < kClimberTolerence
        && Math.abs(drivetrain.getPose().minus(targetPose2d).getY()) < kClimberTolerence;
  }

  @Logged
  public ChassisSpeeds calculateTowerAlignSpeeds() {
    Pose2d targetPose2d;
    if (getAlliance() == Alliance.Red) {
      targetPose2d = kMirrorClimbRed;
    } else {
      targetPose2d = kMirrorClimbBlue;
    }
    Pose2d drivePose = drivetrain.getPose();

    double xDisplacement = targetPose2d.getX() - drivePose.getX();

    double yDisplacement = targetPose2d.getY() - drivePose.getY();

    double xSpeed = 0;
    double ySpeed = 0;

    if (Math.abs(xDisplacement) > kClimberTolerence) {
      xSpeed = (xDisplacement);
    } else {
      if (Math.abs(yDisplacement) > kClimberTolerence) {
        ySpeed = (yDisplacement);
      }
    }
    return new ChassisSpeeds(xSpeed, ySpeed, targetPose2d.getRotation().getRotations());
  }

  @Logged
  public double calculateSafeTrenchXVelocity() {
    if (getTrenchCenter(getClosestTrench(drivetrain.getPose())).getX()
        > drivetrain.getPose().getX()) {
      return -0.2;
    } else {
      return 0.2;
    }
  }

  @Logged
  public Rotation2d targetTOFTrackingError() {
    return getAngleToTargetTOF().minus(drivetrain.getPose().getRotation());
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
  public boolean getInstantShootActive() {
    return !doInstantShoot
        || (doInstantShoot && getActiveShootingPhase())
        || (getShootingTarget(drivetrain.getPose()) == PointingUtil.getFunnlingPoint1()
            || getShootingTarget(drivetrain.getPose()) == PointingUtil.getFunnlingPoint2());
  }

  @Logged
  public boolean getActiveShootingPhase() {
    // double time = Timer.getFPGATimestamp();
    double time = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    var optionalAlliance = DriverStation.getAlliance();
    if (optionalAlliance.isEmpty()) return true;
    var alliance = optionalAlliance.get();

    boolean wonAuto;
    if (gameData.length() == 0) return true;
    switch (gameData.charAt(0)) {
      case 'B':
        wonAuto = alliance.equals(Alliance.Blue);
        break;
      case 'R':
        wonAuto = alliance.equals(Alliance.Red);
        break;
      default:
        return true;
    }
    // 0.55 is the minimum fuel processing time
    time = time - (tofMap.get(shooter.getDistance().in(Meters)) + 0.55);
    if (wonAuto
        && ((130 < time && time < 140)
            || (80 < time && time < 105)
            || (30 < time && time < 55)
            || (time < 30))) {
      return true;
    } else if (!wonAuto
        && ((130 < time && time < 140)
            || (105 < time && time < 130)
            || (55 < time && time < 80)
            || (time < 30))) {
      return true;
    } else {
      return false;
    }
  }

  @Logged
  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

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
