package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter.ShooterLUT.ShooterSetpoint;
import frc.robot.utils.TunableNumber;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

@Logged
public class Shooter extends SubsystemBase implements AutoCloseable {
  protected final Supplier<Pose2d> drivetrainPose;
  protected final Supplier<Pose2d> simPose;
  protected final Supplier<ChassisSpeeds> chassisSpeeds;

  protected final TalonFX flywheelMotor;
  protected final VelocityVoltage flywheelVelocityRequest;
  protected DCMotorSim flywheelSim = null;
  protected TalonFXSimState flywheelMotorSim;

  protected final TalonFX hoodMotor;
  protected final DigitalInput hoodLimitSwitch;
  protected DCMotorSim hoodSim = null;
  protected TalonFXSimState hoodMotorSim;

  protected TunableNumber tunableFlywheelS;
  protected TunableNumber tunableFlywheelA;
  protected TunableNumber tunableFlywheelV;
  protected TunableNumber tunableFlywheelP;
  protected TunableNumber tunableFlywheelI;
  protected TunableNumber tunableFlywheelD;

  protected TunableNumber tunableHoodG;
  protected TunableNumber tunableHoodS;
  protected TunableNumber tunableHoodA;
  protected TunableNumber tunableHoodV;
  protected TunableNumber tunableHoodP;
  protected TunableNumber tunableHoodI;
  protected TunableNumber tunableHoodD;
  protected TunableNumber tunableHoodGravOffset;

  protected TunableNumber tunableHoodAngle;
  protected TunableNumber tunableShooterSpeed;

  public Shooter(
      Supplier<Pose2d> drivetrainPose,
      Supplier<Pose2d> simPose,
      Supplier<ChassisSpeeds> chassisSpeeds,
      TalonFX flywheelMotor,
      TalonFX hoodMotor) {
    this.drivetrainPose = drivetrainPose;
    this.simPose = simPose;
    this.chassisSpeeds = chassisSpeeds;
    this.tunableFlywheelS = new TunableNumber("tunablekS", kFlywheelS);
    this.tunableFlywheelA = new TunableNumber("tunablekA", kFlywheelA);
    this.tunableFlywheelV = new TunableNumber("tunablekV", kFlywheelV);
    this.tunableFlywheelP = new TunableNumber("tunablekP", kFlywheelP);
    this.tunableFlywheelI = new TunableNumber("tunablekI", kFlywheelI);
    this.tunableFlywheelD = new TunableNumber("tunablekD", kFlywheelD);

    this.tunableHoodG = new TunableNumber("tunableHoodG", kHoodG);
    this.tunableHoodS = new TunableNumber("tunablekHoodS", kHoodS);
    this.tunableHoodA = new TunableNumber("tunablekHoodA", kHoodA);
    this.tunableHoodV = new TunableNumber("tunablekHoodV", kHoodV);
    this.tunableHoodP = new TunableNumber("tunablekHoodP", kHoodP);
    this.tunableHoodI = new TunableNumber("tunablekHoodI", kHoodI);
    this.tunableHoodD = new TunableNumber("tunablekHoodD", kHoodD);
    this.tunableHoodGravOffset = new TunableNumber("gravoffset", -05);

    this.tunableHoodAngle = new TunableNumber("Hood angle", 5);
    this.tunableShooterSpeed = new TunableNumber("Flywheel speed", 0);

    this.hoodLimitSwitch = new DigitalInput(kHoodLimitSwitchId);

    this.flywheelMotor = flywheelMotor;
    this.flywheelMotor.getConfigurator().apply(kFlyWheelConfig);
    flywheelVelocityRequest = new VelocityVoltage(RotationsPerSecond.zero()).withEnableFOC(true);

    this.hoodMotor = hoodMotor;
    hoodMotor.getConfigurator().apply(kHoodConfig);

    if (RobotBase.isReal()) return;

    this.flywheelMotorSim = this.flywheelMotor.getSimState();
    this.flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                kFlywheelMotor, kFlywheelMOI.in(KilogramSquareMeters), kFlywheelGearRatio),
            kFlywheelMotor);

    this.hoodMotorSim = this.hoodMotor.getSimState();
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
  public double getFlywheelVelocityRps() {
    return flywheelMotor.getVelocity().getValue().in(RotationsPerSecond);
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
  public Pose2d getShooterPose() {
    return getShooterPose(drivetrainPose.get());
  }

  public static Pose2d getShooterPose(Pose2d robotPose) {
    var rotatedShooter = kShooterLocation.rotateBy(robotPose.getRotation());
    return new Pose2d(
        rotatedShooter.getX() + robotPose.getX(),
        rotatedShooter.getY() + robotPose.getY(),
        rotatedShooter.getRotation());
  }

  ShooterSetpoint lastShooterSetpoint = new ShooterSetpoint(MetersPerSecond.zero(), Degrees.of(5));
  /**
   * @return The speed and position of the shooter to shoot into the hub.
   */
  public ShooterSetpoint getCurrentSetpoint() {
    var setpoint =
        ShooterLUT.generateShootOnTheMoveSetpoint(
            drivetrainPose.get(),
            ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds.get(), drivetrainPose.get().getRotation()));
    if (setpoint.isEmpty()) {
      return lastShooterSetpoint;
    }
    lastShooterSetpoint = setpoint.get().shooterSetpoint();
    return lastShooterSetpoint;
  }

  /**
   * @return Distance from the shooter to the hub
   */
  public static Distance getDistance(Pose2d robotPose) {
    return Meters.of(
        Math.sqrt(
            Math.pow(getRelativeHubLocation(robotPose).getX(), 2)
                + Math.pow(getRelativeHubLocation(robotPose).getY(), 2)));
  }

  public LinearVelocity getCurrentSetpointMPS() {
    return getCurrentSetpoint().flywheelSurfaceSpeed();
  }

  public Angle getCurrentSetpointHoodAngle() {
    return getCurrentSetpoint().rotation();
  }

  /**
   * @return Distance from the shooter to the hub
   */
  public Distance getDistance() {
    return getDistance(drivetrainPose.get());
  }

  public Command sinHoodTest() {
    return run(() -> setHoodAngleInternal(sinHoodMath()));
  }

  public Command sinFlywheelTest() {
    return run(() -> setFlywheelVelocityInternal(sinFlywheelMath()));
  }

  public Angle sinHoodMath() {
    return Degrees.of(Math.sin(Timer.getTimestamp()) * 18 + 27);
  }

  public AngularVelocity sinFlywheelMath() {
    return RotationsPerSecond.of(Math.sin(Timer.getTimestamp()) * 20 + 40);
  }

  public static Pose2d getRelativeHubLocation(Pose2d robotPose) {
    Pose2d pose = getShooterPose(robotPose);
    return kHubLocation.relativeTo(pose);
  }

  public Pose2d getRelativeHubLocation() {
    return getRelativeHubLocation(drivetrainPose.get());
  }

  /**
   * @param velocity Flywheel surface velocity
   * @return Flywheel angular velocity
   */
  public AngularVelocity convertLinearVelocityToAngular(LinearVelocity velocity) {
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

    // return run(() -> {
    //       setFlywheelVelocityInternal(RotationsPerSecond.of(tunableShooterSpeed.get()));
    //       setHoodAngleInternal(Degrees.of(tunableHoodAngle.get()));
    //     })
    //     .withName("Shooter Tuning");
    return run(() -> {
          setFlywheelVelocityInternal(
              RotationsPerSecond.of(
                  getCurrentSetpoint().flywheelSurfaceSpeed().in(MetersPerSecond)));
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
    return convertLinearVelocityToAngular(getCurrentSetpoint().flywheelSurfaceSpeed());
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
  protected void setFlywheelVelocityInternal(AngularVelocity velocity) {
    flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(velocity));
  }

  /**
   * @return The position of the hood
   */
  public Angle getHoodPosition() {
    return hoodMotor.getPosition().getValue();
  }

  /**
   * @return The position of the hood
   */
  public double getHoodPositionDegrees() {
    return hoodMotor.getPosition().getValue().in(Degrees);
  }

  /**
   * @return Command to slowly push the hood against the limit switch
   */
  public Command homeHood() {
    Command command = run(() -> hoodMotor.set(0.01)).until(hoodLimitSwitch::get);
    command.addRequirements(this);
    return command;
  }

  public Vector2 calculateShooterVelocityTranslational() {
    return new Vector2(
        chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond);
  }

  /**
   * @return The hoods position setpoint
   */
  public double getHoodClosedLoopReference() {
    return Rotations.of(hoodMotor.getClosedLoopReference().getValue()).in(Degrees);
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
                Volts.of(10),
                null,
                state -> SignalLogger.writeString("Hood sysid", state.toString())),
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
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6.5),
                null,
                state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())),
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
  protected void flywheelVoltageDrive(Voltage voltage) {
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

  public Command zeroHood() {
    return runOnce(() -> hoodMotor.setPosition(Degrees.of(5)));
  }

  /**
   * @param position Position to set the hood to
   */
  protected void setHoodAngleInternal(Angle position) {
    hoodMotor.setControl(request.withPosition(position));
  }

  public double[] getTuneableSVAPID() {
    return new double[] {
      tunableFlywheelS.get(),
      tunableFlywheelV.get(),
      tunableFlywheelA.get(),
      tunableFlywheelP.get(),
      tunableFlywheelI.get(),
      tunableFlywheelD.get()
    };
  }

  private int timeLastBall = 0;

  private TalonFXConfiguration generateTunableFlywheelConfig() {
    return generateFlywheelConfig(
        tunableFlywheelS.get(),
        tunableFlywheelA.get(),
        tunableFlywheelV.get(),
        tunableFlywheelP.get(),
        tunableFlywheelI.get(),
        tunableFlywheelD.get());
  }

  private TalonFXConfiguration generateTunableHoodConfig() {
    return generateHoodConfig(
        tunableHoodG.get(),
        tunableHoodS.get(),
        tunableHoodA.get(),
        tunableHoodV.get(),
        tunableHoodP.get(),
        tunableHoodI.get(),
        tunableHoodD.get(),
        tunableHoodGravOffset.get());
  }

  public void detectTunableFlywheelChanges() {
    if (tunableFlywheelS.hasChanged()
        || tunableFlywheelA.hasChanged()
        || tunableFlywheelV.hasChanged()
        || tunableFlywheelP.hasChanged()
        || tunableFlywheelI.hasChanged()
        || tunableFlywheelD.hasChanged()) {
      this.flywheelMotor.getConfigurator().apply(generateTunableFlywheelConfig());
    }
  }

  public void detectTunableHoodChanges() {
    if (tunableHoodG.hasChanged()
        || tunableHoodS.hasChanged()
        || tunableHoodA.hasChanged()
        || tunableHoodV.hasChanged()
        || tunableHoodP.hasChanged()
        || tunableHoodI.hasChanged()
        || tunableHoodD.hasChanged()
        || tunableHoodGravOffset.hasChanged()) {
      this.hoodMotor.getConfigurator().apply(generateTunableHoodConfig());
    }
  }

  @Override
  public void periodic() {
    if (hoodLimitSwitch.get()
        && RobotBase.isReal()
        && isWithinTolerance(getHoodPosition(), Degrees.of(90), Degrees.of(0.01))) {
      hoodMotor.setPosition(Degrees.of(90));
    }
    // var controller = new CommandXboxController(0);
    // if (timeLastBall >= 15 && controller.rightBumper().getAsBoolean()) {
    //   this.shootSimulatedProjectile();
    //   timeLastBall = 0;
    // } else {
    //   timeLastBall += 1;
    // }
    detectTunableFlywheelChanges();
    detectTunableHoodChanges();
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

    hoodMotorSim.setRawRotorPosition(hoodSim.getAngularPosition().times(kHoodGearRatio));
    hoodMotorSim.setRotorVelocity(hoodSim.getAngularVelocity().times(kHoodGearRatio));
    hoodMotorSim.setRotorAcceleration(hoodSim.getAngularAcceleration().times(kHoodGearRatio));
  }

  /** Shoots a fuel in maple sim */
  public void shootSimulatedProjectile() {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                simPose.get().getTranslation(),
                kMapleSimShooterLocationNotJankAtAll.getTranslation(),
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    chassisSpeeds.get(), drivetrainPose.get().getRotation()),
                kMapleSimSHooterRotationAlsoNotJank.plus(simPose.get().getRotation()),
                kShooterHeight,
                getCurrentSetpoint()
                    .flywheelSurfaceSpeed()
                    .times(kEstimatedFlywheelSpeedToFuelSpeed),
                getCurrentSetpoint().rotation().plus(Degrees.of(90))));
  }
}
