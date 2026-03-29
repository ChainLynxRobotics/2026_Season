package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Trench;
import frc.robot.subsystems.Shooter.ShooterLUT.ShooterSetpoint;
import frc.robot.utils.SpikeDetector;
import frc.robot.utils.TunableNumber;
import java.util.function.BooleanSupplier;
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
  protected final TalonFX flywheelFollower;
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
  protected TunableNumber tunableLUTMult;

  protected BooleanSupplier hasVisionPose;

  public SpikeDetector flywheelSpikeDetector = new SpikeDetector(0.1, 0.0, 1.5, true);

  public Timer flywheelSpikeTimer = new Timer();

  private boolean spiking = false;

  public boolean isShooting = false;

  protected Supplier<CommandXboxController> driveController;

  public Shooter(
      Supplier<Pose2d> drivetrainPose,
      Supplier<Pose2d> simPose,
      Supplier<ChassisSpeeds> chassisSpeeds,
      TalonFX flywheelMotor,
      TalonFX flywheelFollower,
      TalonFX hoodMotor,
      BooleanSupplier hasVisionPose,
      Supplier<CommandXboxController> driveController) {
    this.drivetrainPose = drivetrainPose;
    this.simPose = simPose;
    this.chassisSpeeds = chassisSpeeds;
    this.driveController = driveController;
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

    this.tunableLUTMult = new TunableNumber("tunableLUTMult", 1.09);
    this.tunableHoodGravOffset = new TunableNumber("gravoffset", -05);

    this.tunableHoodAngle = new TunableNumber("Hood angle", 8);
    this.tunableShooterSpeed = new TunableNumber("Flywheel speed", 0);

    this.hoodLimitSwitch = new DigitalInput(kHoodLimitSwitchId);

    this.flywheelMotor = flywheelMotor;
    this.flywheelFollower = flywheelFollower;
    this.flywheelFollower.setControl(new Follower(kFlywheelCANId, MotorAlignmentValue.Opposed));
    this.flywheelMotor.getConfigurator().apply(kFlyWheelConfig);
    this.flywheelFollower.getConfigurator().apply(kFlyWheelConfig);
    flywheelVelocityRequest = new VelocityVoltage(RotationsPerSecond.zero()).withEnableFOC(true);

    this.hoodMotor = hoodMotor;
    hoodMotor.getConfigurator().apply(kHoodConfig);

    this.hasVisionPose = hasVisionPose;

    hoodMotor.setPosition(Degrees.of(8));

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

  public double getFlywheelStator() {
    return flywheelMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getFlywheelFollowerStator() {
    return flywheelFollower.getStatorCurrent().getValueAsDouble();
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

  ShooterSetpoint lastShooterSetpoint = new ShooterSetpoint(MetersPerSecond.zero(), Degrees.of(8));
  /**
   * @return The speed and position of the shooter to shoot into the hub.
   */
  public ShooterSetpoint getCurrentSetpoint(Pose2d targetPose) {
    var setpoint =
        ShooterLUT.generateShootOnTheMoveSetpoint(
            drivetrainPose.get(),
            ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds.get(), drivetrainPose.get().getRotation()),
            targetPose);
    if (setpoint.isEmpty()) {
      return lastShooterSetpoint;
    }
    lastShooterSetpoint = setpoint.get().shooterSetpoint();
    return lastShooterSetpoint;
  }

  /**
   * @return Distance from the shooter to the hub
   */
  public static Distance getDistanceToHub(Pose2d robotPose) {
    // return Meters.of(3);
    return getDistanceToPose(robotPose, getHubLocation2d());
  }

  public static Distance getDistanceToPose(Pose2d robotPose, Pose2d targetPose) {
    return Meters.of(
        Math.sqrt(
            Math.pow(getRelativeLocation(robotPose, targetPose).getX(), 2)
                + Math.pow(getRelativeLocation(robotPose, targetPose).getY(), 2)));
  }

  /**
   * @return Distance from the shooter to the hub
   */
  public Distance getDistance() {
    return getDistanceToHub(drivetrainPose.get());
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
    return RotationsPerSecond.of(Math.sin(Timer.getTimestamp()) * 15 + 30);
  }

  public static Pose2d getRelativeHubLocation(Pose2d robotPose) {
    return getRelativeLocation(robotPose, getHubLocation2d());
  }

  public static Pose2d getRelativeLocation(Pose2d robotPose, Pose2d targetPose) {
    Pose2d pose = getShooterPose(robotPose);
    return targetPose.relativeTo(pose);
  }

  public Pose2d getRelativeHubLocation() {
    return getRelativeHubLocation(drivetrainPose.get());
  }

  public Pose3d getHoodPose() {
    return new Pose3d(
        -0.2,
        -0.096,
        0.435,
        new Rotation3d(Degrees.of(8 - getHoodPositionDegrees()), Degrees.of(0), Degrees.of(0)));
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

  public Time timeSinceLastBall() {
    flywheelSpikeTimer.start();
    this.spiking = flywheelSpikeDetector.update(getFlywheelVelocityRps());
    if (spiking) {
      flywheelSpikeTimer.reset();
    }
    return Seconds.of(flywheelSpikeTimer.get());
  }

  public boolean isFlywheelSpiking() {
    return spiking;
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
    // TODO: REPLACE HUB WITH VARIABLE TARGET
    // return run(() -> {
    //       setFlywheelVelocityInternal(RotationsPerSecond.of(tunableShooterSpeed.get()));
    //       setHoodAngleInternal(Degrees.of(tunableHoodAngle.get()));
    //     })
    //     .withName("Shooter Tuning");
    // TODO: REPLACE HUB WITH VARIABLE TARGET
    // Joe: The LUT returns flywheelSurfaceSpeed as m/s, but setFlywheelVelocityInternal expects
    // RPS — wrapping m/s in RotationsPerSecond.of() won't give the right value, right? Should we
    // use convertLinearVelocityToAngular() here?
    // if (!hasVisionPose.getAsBoolean()) {
    //   setFlywheelVelocityInternal(RotationsPerSecond.of(60).times(tunableLUTMult.get()));
    //   setHoodAngleInternal(calculateHoodAngle());
    // }
    return run(() -> {
          setFlywheelVelocityInternal(
              RotationsPerSecond.of(
                      getCurrentSetpoint(getHubLocation2d())
                          .flywheelSurfaceSpeed()
                          .in(MetersPerSecond))
                  .times(tunableLUTMult.get()));
          setHoodAngleInternal(calculateHoodAngle());
        })
        .withName("Shooter control");
  }

  public Command raiseIntakeOscillate() {
    return repeatingSequence(
        runOnce(() -> hoodMotor.setControl(request.withPosition(Degrees.of(45)))),
        waitSeconds(1),
        runOnce(() -> hoodMotor.setControl(request.withPosition(Degrees.of(70)))),
        waitSeconds(1));
  }

  /**
   * @param velocity The velocity to set the flywheel to
   * @return The command
   */
  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return runOnce(() -> setFlywheelVelocityInternal(velocity)).withName("Flywheel velocity");
  }

  // TODO: REPLACE HUB WITH VARIABLE TARGET
  /**
   * @return The target angular velocity for the flywheel
   */
  public AngularVelocity targetVelocity() {
    return convertLinearVelocityToAngular(
        getCurrentSetpoint(getHubLocation2d()).flywheelSurfaceSpeed());
  }

  // TODO: REPLACE HUB WITH VARIABLE TARGET
  /**
   * @return The target linear velocity for the flywheel
   */
  public LinearVelocity targetLinearVelocity() {
    return getCurrentSetpoint(getHubLocation2d()).flywheelSurfaceSpeed();
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
                .until(() -> getFlywheelVelocity().gt(RotationsPerSecond.of(100))),
            routine
                .quasistatic(Direction.kReverse)
                .until(() -> getFlywheelVelocity().lt(RotationsPerSecond.zero())),
            routine
                .dynamic(Direction.kForward)
                .until(() -> getFlywheelVelocity().gt(RotationsPerSecond.of(100))),
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
      this.flywheelFollower.getConfigurator().apply(generateTunableFlywheelConfig());
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

  public AngularVelocity rollingVelocityAverage = RotationsPerSecond.zero();
  private double averageWindow = 0.25;

  @Override
  public void periodic() {
    rollingVelocityAverage =
        rollingVelocityAverage
            .times(1 - averageWindow)
            .plus(getFlywheelVelocity().times(averageWindow));
    if (hoodLimitSwitch.get()
        && RobotBase.isReal()
        && isWithinTolerance(getHoodPosition(), Degrees.of(90), Degrees.of(0.01))) {
      hoodMotor.setPosition(Degrees.of(90));
    }

    detectTunableFlywheelChanges();
    detectTunableHoodChanges();
    ballStuck = ballStuckDebouncer.calculate(hasAStuckBallInternal());
  }

  private boolean ballStuck = false;
  private Debouncer ballStuckDebouncer = new Debouncer(0.25);

  public boolean hasAStuckBallInternal() {
    var acceleration = flywheelMotor.getAcceleration().getValue();
    var notAccelerating =
        acceleration.lt(RotationsPerSecondPerSecond.of(5))
            && acceleration.gt(RotationsPerSecondPerSecond.of(-5));
    var notMoving =
        rollingVelocityAverage.lt(RotationsPerSecond.of(5))
            && rollingVelocityAverage.gt(RotationsPerSecond.of(-5));
    return notAccelerating && notMoving && DriverStation.isEnabled();
  }

  public boolean hasAStuckBall() {
    return ballStuck;
  }

  public Angle calculateHoodAngle() {
    Angle setpoint = getCurrentSetpoint(getHubLocation2d()).rotation();
    
    if (!isShooting) {
      return Degrees.of(8);
    }
    if (setpoint.gt(Degrees.of(45))) {
      return Degrees.of(45);
    }

    return setpoint;
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

  // TODO: REPLACE HUB WITH VARIABLE TARGET
  /** Shoots a fuel in maple sim */
  public void shootSimulatedProjectile() {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                simPose.get().getTranslation(),
                new Translation2d(0.213, -0.213),
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    chassisSpeeds.get(), drivetrainPose.get().getRotation()),
                kMapleSimSHooterRotationAlsoNotJank.plus(simPose.get().getRotation()),
                kShooterHeight,
                getCurrentSetpoint(getHubLocation2d())
                    .flywheelSurfaceSpeed()
                    .times(kEstimatedFlywheelSpeedToFuelSpeed),
                getCurrentSetpoint(getHubLocation2d()).rotation().plus(Degrees.of(90))));
  }
}
