package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter.ShooterLUT.ShooterSetpoint;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

@Logged
public class Shooter extends SubsystemBase implements AutoCloseable {
  private final Supplier<Pose2d> drivetrainPose;
  private final Supplier<Pose2d> simPose;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final TalonFX flywheelMotor;
  private final VelocityVoltage flywheelMotionMagic;
  public DCMotorSim flywheelSim = null;
  public TalonFXSimState flywheelMotorSim;

  private final TalonFX hoodMotor;
  private final DigitalInput hoodLimitSwitch;
  public DCMotorSim hoodSim = null;
  public TalonFXSimState hoodMotorSim;

  public Shooter(
      Supplier<Pose2d> drivetrainPose,
      Supplier<Pose2d> simPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.drivetrainPose = drivetrainPose;
    this.simPose = simPose;
    this.chassisSpeeds = chassisSpeeds;

    this.hoodLimitSwitch = new DigitalInput(kHoodLimitSwitchId);

    this.flywheelMotor = new TalonFX(kFlywheelCANId);
    flywheelMotor.getConfigurator().apply(kFlyWheelConfig);
    flywheelMotionMagic = new VelocityVoltage(RotationsPerSecond.zero()).withEnableFOC(true);

    this.hoodMotor = new TalonFX(kHoodCANId);
    hoodMotor.getConfigurator().apply(kHoodConfig);

    if (RobotBase.isReal()) return;

    this.flywheelMotorSim = flywheelMotor.getSimState();
    this.flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                kFlywheelMotor, kFlywheelMOI.in(KilogramSquareMeters), kFlywheelGearRatio),
            kFlywheelMotor);

    this.hoodMotorSim = hoodMotor.getSimState();
    this.hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                kHoodMotor, kHoodMOI.in(KilogramSquareMeters), kHoodGearRatio),
            kHoodMotor);
  }

  /** Stops the motors */
  public void stop() {
    flywheelMotor.stopMotor();
    hoodMotor.stopMotor();
  }

  @Override

  /** Turns the Motors off */
  public void close() {

    flywheelMotor.close();
    hoodMotor.close();
  }
  /**
   * Gets the voltage of the hood motor
   *
   * @return Voltage
   */
  public Voltage getHoodVoltage() {
    return hoodMotor.getMotorVoltage().getValue();
  }

  /**
   * Gets the current running command name
   *
   * @return currentCommand as String
   */
  public String currentCommand() {
    if (this.getCurrentCommand() != null) return this.getCurrentCommand().getName();

    return "";
  }

  /**
   * gets the position of the flywheel
   *
   * @return Position of flywheel
   */
  public Angle getFlywheelPosition() {
    return flywheelMotor.getPosition().getValue();
  }
  /**
   * Get the setpoint of the flywheel
   *
   * @return the setpoint of the flywheel
   */
  public double getFlywheelSetpoint() {
    return flywheelMotor.getClosedLoopReference().getValueAsDouble();
  }
  /**
   * gets position of the flywheel in rotations
   *
   * @return position of the flywheel in rotations
   */
  public double getFlywheelPositionRotations() {
    return getFlywheelPosition().in(Rotations);
  }
  /**
   * Gets flywheel Velocity
   *
   * @return Flywheel veclotiy
   */
  public AngularVelocity getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValue();
  }
  /**
   * Gets flywheel veloctiy in rps
   *
   * @return flywheel Velocity in RotationsPerSecond
   */
  public double getFlywheelVelocityRpS() {
    return getFlywheelVelocity().in(RotationsPerSecond);
  }
  /**
   * Gets flywheel Voltage
   *
   * @return Returns the Voltage in the flywheel
   */
  public Voltage getFlywheelVoltage() {
    return flywheelMotor.getMotorVoltage().getValue();
  }
  /**
   * Gets the control Mode of the Flywheel
   *
   * @return Control Mode of the Flywheel
   */
  public ControlModeValue getFlywheelControlMode() {
    return flywheelMotor.getControlMode().getValue();
  }
  /**
   * Converts Robot Pose to Shooter Pose
   *
   * @param robotPose Input robot pose
   * @return Shooter pose
   */
  public Pose3d convertRobotPoseToShooterPose(Pose3d robotPose) {
    return kShooterLocation.transformBy(new Transform3d(new Pose3d(), robotPose));
  }
  /**
   * Gets the setpoint of the shooter
   *
   * @return the speed and position of the shooter to shoot into the hub.
   */
  public ShooterSetpoint getCurrentSetpoint() {
    var shooterFieldLocation = convertRobotPoseToShooterPose(new Pose3d(drivetrainPose.get()));
    var shooterDistance =
        Meters.of(
            Math.sqrt(
                Math.pow(shooterFieldLocation.getX() - kHubLocation.getX(), 2)
                    + Math.pow(shooterFieldLocation.getY() - kHubLocation.getY(), 2)));

    return ShooterLUT.getSpeedAndRotation(shooterDistance);
  }
  /**
   * Gets the distance of the robot
   *
   * @return Distance of the robot in Meters per seconds
   */
  public Distance getDistance() {
    var shooterFieldLocation =
        kShooterLocation.transformBy(
            new Transform3d(new Pose3d(), new Pose3d(drivetrainPose.get())));
    return Meters.of(
        Math.sqrt(
            Math.pow(shooterFieldLocation.getX() - kHubLocation.getX(), 2)
                + Math.pow(shooterFieldLocation.getY() - kHubLocation.getY(), 2)));
  }
  /**
   * Converts linear velocity to angular velocity
   * @param velocity the velocity in meters/sec
   * @return the velocity in rotations/sec
   */
  public AngularVelocity convertLinearVelocityToAngula(LinearVelocity velocity) {
    return RotationsPerSecond.of(
        velocity.in(MetersPerSecond) / (kFlywheelRadius.in(Meters) * 2 * Math.PI));
  }
  /**
   * Converts angular velocity to linear velocity
   * @param velocity the velocity in rotations/sec
   * @return the velocity in meters/sec
   */
  public LinearVelocity convertAngularVelocityToLinear(AngularVelocity velocity) {
    return MetersPerSecond.of(
        velocity.in(RotationsPerSecond) * (kFlywheelRadius.in(Meters) * 2 * Math.PI));
  }
  /**
   * Sets flywheel and shooter to the postion and velocity it need from current position.
   * @return the run command.
   */
  public Command runShooterControl() {
    return run(() -> {
          setFlywheelVelocityInternal(
              convertLinearVelocityToAngula(getCurrentSetpoint().flywheelSurfaceSpeed()));
          setHoodAngleInternal(getSetpointRotation());
        })
        .withName("Shooter control");
  }
  /**
   * Sets the velocity of the flywheel
   * @param velocity The velocity to set the flywheel to
   * @return The command to run
   */
  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return runOnce(() -> setFlywheelVelocityInternal(velocity)).withName("Flywheel velocity");
  }
  /**
   * Gets the target velocity of the flywheel
   * @return The surface speed of the flywheel setpoint
   */
  public AngularVelocity targetVelocity() {
    return convertLinearVelocityToAngula(getCurrentSetpoint().flywheelSurfaceSpeed());
  }
  /**
   * Gets the target linear velocity of the flywheel
   * @return The surface speed of the flywheel
   */
  public LinearVelocity targetLinearVelocity() {
    return getCurrentSetpoint().flywheelSurfaceSpeed();
  }
  /**
   * Sets the target linear velocity of the flywheel
   * @param velocity The target velocity
   */
  private void setFlywheelVelocityInternal(AngularVelocity velocity) {
    flywheelMotor.setControl(flywheelMotionMagic.withVelocity(velocity));
  }
  /**
   * Gets the position of the hood
   * @return The position of the hood
   */
  public Angle getHoodPosition() {
    return hoodMotor.getPosition().getValue();
  }
  /**
   * Calibrate the position of the hood
   * @return The command to run
   */
  public Command homeHood() {
    Command command = run(() -> hoodMotor.set(0.01)).until(hoodLimitSwitch::get);
    command.addRequirements(this);
    return command;
  }
  /**
   * Gets the value of the hood's closed loop reference
   * @return status signal
   */
  public double getHoodClosedLoopReference() {
    return hoodMotor.getClosedLoopReference().getValue();
  }
  /**
   * Sets hood Voltage
   * @param voltage The voltage.
   */
  public void hoodVoltageDrive(Voltage voltage) {
    hoodMotor.setControl(new VoltageOut(voltage));
  }
  /**
   * Gets angular velocity of the hood
   *
   * @return The angular velocity of the hood in rotations
   */
  public AngularVelocity getHoodVelocity() {
    return hoodMotor.getVelocity().getValue();
  }
  /**
   * Gets position of the hood motor in rotations
   *
   * @return the position of the hood in rotations
   */
  public double getHoodPositionRotations() {
    return getHoodPosition().in(Rotations);
  }
  /**
   * Gets the velocity of the hood motor as a double
   *
   * @return velocity of the hood in rotations/sec
   */
  public double getHoodVelocityRotationsPerSecond() {
    return getHoodVelocity().in(RotationsPerSecond) + Math.random() * 0.0001;
  }
  /**
   * The sysid of the hood
   * @return sequence of routines
   */
  public Command hoodSysid() {
    var routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(6),
                null,
                state -> SignalLogger.writeString("Flywheel sysid", state.toString())),
            new SysIdRoutine.Mechanism(this::hoodVoltageDrive, null, this));
    return sequence(
            routine
                .quasistatic(Direction.kForward)
                .until(() -> getHoodPosition().gt(Degrees.of(70))),
            routine
                .quasistatic(Direction.kReverse)
                .until(() -> getHoodPosition().lt(Degrees.of(10))),
            routine.dynamic(Direction.kForward).until(() -> getHoodPosition().gt(Degrees.of(70))),
            routine.dynamic(Direction.kReverse).until(() -> getHoodPosition().lt(Degrees.of(10))))
        .withName("Hood sysid");
  }
  /**
   * run Sysid on the flywheel
   * @return a sequence of routines
   */
  public Command flywheelSysid() {
    var routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(6.5), null, null),
            new SysIdRoutine.Mechanism(this::flywheelVoltageDrive, null, this));
    return sequence(
            routine
                .quasistatic(Direction.kForward)
                .until(() -> getFlywheelVelocity().gt(RotationsPerSecond.of(50))),
            routine
                .quasistatic(Direction.kReverse)
                .until(() -> getFlywheelVelocity().lt(RotationsPerSecond.zero())),
            routine
                .dynamic(Direction.kForward)
                .until(() -> getFlywheelVelocity().gt(RotationsPerSecond.of(50))),
            routine
                .dynamic(Direction.kReverse)
                .until(() -> getFlywheelVelocity().lt(RotationsPerSecond.zero())))
        .withName("Flywheel sysid");
  }
  /**
   * Sets target voltage of the flywheel motor
   * @param voltage The voltage.
   */
  private void flywheelVoltageDrive(Voltage voltage) {
    flywheelMotor.setControl(new VoltageOut(voltage));
  }

  final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);
  /**
   * Sets the hood angle
   * @param position position of the hood you want to set it to
   * @return run hood to position
   */
  public Command setHoodAngle(Angle position) {
    return runOnce(() -> setHoodAngleInternal(position));
  }
  /**
   * Sets the internal hood angle
   * @param position postion of the hood angle
   */
  private void setHoodAngleInternal(Angle position) {
    hoodMotor.setControl(request.withPosition(position));
  }

  private int timeLastBall = 0;

  @Override
  public void periodic() {
    if (hoodLimitSwitch.get()
        && RobotBase.isReal()
        && isWithinTolerance(getHoodPosition(), Degrees.of(90), Degrees.of(0.01))) {
      hoodMotor.setPosition(Degrees.of(90));
    }
    if (timeLastBall == 25) {
      this.shootSimulatedProjectile();
      timeLastBall = 0;
    } else {
      timeLastBall += 1;
    }
  }

  @Override
  public void simulationPeriodic() {
    flywheelMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    flywheelSim.setInputVoltage(flywheelMotorSim.getMotorVoltage());

    flywheelSim.update(kDT.in(Seconds));

    flywheelMotorSim.setRawRotorPosition(flywheelSim.getAngularPosition());
    flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity());
    flywheelMotorSim.setRotorAcceleration(flywheelSim.getAngularAcceleration());

    hoodMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    hoodSim.setInputVoltage(hoodMotorSim.getMotorVoltage());

    hoodSim.update(kDT.in(Seconds));

    hoodMotorSim.setRawRotorPosition(hoodSim.getAngularPosition());
    hoodMotorSim.setRotorVelocity(hoodSim.getAngularVelocity());
    hoodMotorSim.setRotorAcceleration(hoodSim.getAngularAcceleration());
  }
  /**
   * Gets the rotation of the current setpoint.
   * @return the rotation
   */
  public Angle getSetpointRotation() {
    return getCurrentSetpoint().rotation();
  }

  public void shootSimulatedProjectile() {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                simPose.get().getTranslation(),
                kShooterLocation.getTranslation().toTranslation2d(),
                chassisSpeeds.get(),
                kShooterLocation.getRotation().toRotation2d(),
                kShooterLocation.getMeasureZ(),
                convertAngularVelocityToLinear(
                    getFlywheelVelocity().times(kEstimatedFlywheelSpeedToFuelSpeed)),
                getHoodPosition()));
  }
}
