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
  /** Closes all objects in the class. This should be called at the end of unit tests */
  public void close() {
    flywheelMotor.close();
    hoodMotor.close();
    hoodLimitSwitch.close();
  }

  /**
   * @return applied voltage of the hood motor
   */
  public Voltage getHoodVoltage() {
    return hoodMotor.getMotorVoltage().getValue();
  }

  /**
   * @return name of the current command
   */
  public String currentCommand() {
    if (this.getCurrentCommand() != null) return this.getCurrentCommand().getName();

    return "";
  }

  /**
   * @return Position of flywheel
   */
  public Angle getFlywheelPosition() {
    return flywheelMotor.getPosition().getValue();
  }

  /**
   * @return the setpoint of the flywheel in rotations per second
   */
  public double getFlywheelSetpoint() {
    return flywheelMotor.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * @return Flywheel veclotiy
   */
  public AngularVelocity getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValue();
  }

  /**
   * @return applied voltage of the flywheel motor
   */
  public Voltage getFlywheelVoltage() {
    return flywheelMotor.getMotorVoltage().getValue();
  }

  /**
   * @return Control Mode of the Flywheel
   */
  public ControlModeValue getFlywheelControlMode() {
    return flywheelMotor.getControlMode().getValue();
  }

  /**
   * @param robotPose Robot pose
   * @return Shooter pose
   */
  public Pose3d convertRobotPoseToShooterPose(Pose3d robotPose) {
    return kShooterLocation.transformBy(new Transform3d(new Pose3d(), robotPose));
  }

  /**
   * @return The speed and position of the shooter to shoot into the hub.
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
   * @return Distance from the shooter to the hub
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
   * @param velocity Flywheel surface velocity
   * @return Flywheel angular velocity
   */
  public AngularVelocity convertLinearVelocityToAngula(LinearVelocity velocity) {
    return RotationsPerSecond.of(
        velocity.in(MetersPerSecond) / (kFlywheelRadius.in(Meters) * 2 * Math.PI));
  }

  /**
   * @param velocity Flywheel angular velocity
   * @return Flywheel surface velocity
   */
  public LinearVelocity convertAngularVelocityToLinear(AngularVelocity velocity) {
    return MetersPerSecond.of(
        velocity.in(RotationsPerSecond) * (kFlywheelRadius.in(Meters) * 2 * Math.PI));
  }

  /**
   * @return Command that automatically controls shooting into the hub
   */
  public Command runShooterControl() {
    return run(() -> {
          setFlywheelVelocityInternal(
              convertLinearVelocityToAngula(getCurrentSetpoint().flywheelSurfaceSpeed()));
          setHoodAngleInternal(getCurrentSetpoint().rotation());
        })
        .withName("Shooter control");
  }

  /**
   * @param velocity The velocity to set the flywheel to
   * @return The command
   */
  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return runOnce(() -> setFlywheelVelocityInternal(velocity)).withName("Flywheel velocity");
  }

  /**
   * @return The target angular velocity for the flywheel
   */
  public AngularVelocity targetVelocity() {
    return convertLinearVelocityToAngula(getCurrentSetpoint().flywheelSurfaceSpeed());
  }

  /**
   * @return The target linear velocity for the flywheel
   */
  public LinearVelocity targetLinearVelocity() {
    return getCurrentSetpoint().flywheelSurfaceSpeed();
  }

  /**
   * @param velocity The target velocity of the flywheel
   */
  private void setFlywheelVelocityInternal(AngularVelocity velocity) {
    flywheelMotor.setControl(flywheelMotionMagic.withVelocity(velocity));
  }

  /**
   * @return The position of the hood
   */
  public Angle getHoodPosition() {
    return hoodMotor.getPosition().getValue();
  }

  /**
   * @return Command to slowly push the hood against the limit switch
   */
  public Command homeHood() {
    Command command = run(() -> hoodMotor.set(0.01)).until(hoodLimitSwitch::get);
    command.addRequirements(this);
    return command;
  }

  /**
   * @return The hoods position setpoint
   */
  public double getHoodClosedLoopReference() {
    return hoodMotor.getClosedLoopReference().getValue();
  }

  /**
   * @param voltage Voltage to apply to the hood motor
   */
  public void hoodVoltageDrive(Voltage voltage) {
    hoodMotor.setControl(new VoltageOut(voltage));
  }

  /**
   * @return Velocity of the hood
   */
  public AngularVelocity getHoodVelocity() {
    return hoodMotor.getVelocity().getValue();
  }

  /**
   * @return A command to run a full sysid routine on the hood
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
   * @return A command to run a full sysid routine on the flywheel
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
   * @param voltage Voltage to apply to the flywheel.
   */
  private void flywheelVoltageDrive(Voltage voltage) {
    flywheelMotor.setControl(new VoltageOut(voltage));
  }

  final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);
  /**
   * @param position Position to set the hood to
   * @return Command to run
   */
  public Command setHoodAngle(Angle position) {
    return runOnce(() -> setHoodAngleInternal(position));
  }
  /**
   * @param position Position to set the hood to
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

    flywheelMotorSim.setRawRotorPosition(
        flywheelSim.getAngularPosition().times(kFlywheelGearRatio));
    flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity().times(kFlywheelGearRatio));
    flywheelMotorSim.setRotorAcceleration(
        flywheelSim.getAngularAcceleration().times(kFlywheelGearRatio));

    hoodMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    hoodSim.setInputVoltage(hoodMotorSim.getMotorVoltage());

    hoodSim.update(kDT.in(Seconds));

    hoodMotorSim.setRawRotorPosition(hoodSim.getAngularPosition());
    hoodMotorSim.setRotorVelocity(hoodSim.getAngularVelocity());
    hoodMotorSim.setRotorAcceleration(hoodSim.getAngularAcceleration());
  }

  /** Shoots a fuel in maple sim */
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
