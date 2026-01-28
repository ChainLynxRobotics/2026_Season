// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

@Logged
public class RobotContainer {

  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              // Use open-loop control for drive motors
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle pointAtHub =
      new SwerveRequest.FieldCentricFacingAngle()
          // This pid is vibes for now fyi
          .withHeadingPID(7, 0, 0)
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driveJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final IntakeSubsystem intake = new IntakeSubsystem(new TalonFX(0), new TalonFX(1));

  public final Vision vision = new Vision(drivetrain::passVisionPose, drivetrain::getSimPose);

  public final Shooter shooter =
      new Shooter(
          () -> drivetrain.getState().Pose,
          drivetrain::getSimPose,
          () -> drivetrain.getState().Speeds,
          new TalonFX(ShooterConstants.kFlywheelCANId),
          new TalonFX(ShooterConstants.kHoodCANId));

  public RobotContainer() {
    NamedCommands.registerCommand("goToHub", goToHub());
    NamedCommands.registerCommand("Climb", new PrintCommand("Implement it plz"));
    NamedCommands.registerCommand("Shoot Balls", new PrintCommand("Implement it plz"));
    NamedCommands.registerCommand("Stop Shoot", new PrintCommand("Implement it plz"));
    NamedCommands.registerCommand("Scoop", new PrintCommand("Implement it plz"));
    NamedCommands.registerCommand("Stop Scoop", new PrintCommand("Implement it plz"));

    configureBindings();
    // shooter.setDefaultCommand(shooter.runShooterControl());
    if (Robot.isSimulation()) SimulatedArena.getInstance().resetFieldForAuto();
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
                        -driveJoystick.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driveJoystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driveJoystick
        .y()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    // on y button press rotate robot to angle from getAngleToHub()
                    pointAtHub
                        .withTargetDirection(getAngleToHub())
                        .withVelocityX(
                            -driveJoystick.getLeftY()
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)));

    driveJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveJoystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driveJoystick.getLeftY(), -driveJoystick.getLeftX()))));

    driveJoystick
        .y()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    // on y button press rotate robot to angle from getAngleToHub()
                    new SwerveRequest.FieldCentricFacingAngle()
                        .withTargetDirection(getAngleToHub())
                        .withHeadingPID(7, 0, 0)
                        // ^This pid is vibes for now fyi
                        .withVelocityX(
                            -driveJoystick.getLeftY()
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -driveJoystick.getLeftX()
                                * MaxSpeed))); // Drive left with negative X (left)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driveJoystick.leftBumper().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driveJoystick.leftTrigger().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driveJoystick.rightBumper().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driveJoystick.rightTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press. Commented for sysId uncoment later
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    new Trigger(() -> intakeSimJoystick.getRawButton(1)).onTrue(intake.spin());

    operatorJoystick.a().whileTrue(shooter.hoodSysid());
    operatorJoystick.b().onTrue(shooter.setFlywheelVelocity(RotationsPerSecond.of(20)));
    operatorJoystick.x().onTrue(shooter.setHoodAngle(Degrees.of(60)));
    operatorJoystick.y().onTrue(runOnce(shooter::shootSimulatedProjectile));
  }

  public Command goToHub() {
    return drivetrain.applyRequest(
        () ->
            new SwerveRequest.FieldCentricFacingAngle()
                .withTargetDirection(getAngleToHub())
                .withHeadingPID(7, 0, 0)
                .withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed));
  }

  public Rotation2d getAngleToHub() {
    return new Transform2d(
            new Pose2d(drivetrain.getState().Pose.getTranslation(), new Rotation2d()),
            DrivetrainConstants.kHubLocation.toPose2d())
        .getTranslation()
        .getAngle();
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("0");

    // Simple drive forward auton
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

  public Voltage getSimulatedBatteryVoltage() {
    return SimulatedBattery.getBatteryVoltage();
  }
}
