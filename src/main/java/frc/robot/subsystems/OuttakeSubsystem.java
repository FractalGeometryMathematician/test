package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(OuttakeConstants.motorID, "can");
  private final TalonFXConfiguration motorConfig;
  private final Slot0Configs positionPIDConfigs;
  private final Slot1Configs velocityPIDConfigs;

  final PositionTorqueCurrentFOC posRequest = new PositionTorqueCurrentFOC(Rotations.of(0)).withSlot(0);
  final VelocityTorqueCurrentFOC velRequest = new VelocityTorqueCurrentFOC(RPM.of(0)).withSlot(1);
  final VoltageOut voltageRequest = new VoltageOut(0.0);

  private Per<TorqueUnit, CurrentUnit> motorKt;

  private final CANcoder pivotEncoder = new CANcoder(OuttakeConstants.encoderID, "can");
  private final CANcoderConfiguration encoderConfig;

  private final CANdi hardstop = new CANdi(OuttakeConstants.CANdiID, "can");
  private final CANdiConfiguration candiConfig;

  public OuttakeSubsystem() {
    positionPIDConfigs = new Slot0Configs()
        .withKG(OuttakeConstants.pos_kG)
        .withKS(OuttakeConstants.pos_kS)
        .withKP(OuttakeConstants.pos_kP)
        .withKI(OuttakeConstants.pos_kI)
        .withKD(OuttakeConstants.pos_kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    velocityPIDConfigs = new Slot1Configs()
        .withKG(OuttakeConstants.vel_kG)
        .withKS(OuttakeConstants.vel_kS)
        .withKP(OuttakeConstants.vel_kP)
        .withKI(OuttakeConstants.vel_kI)
        .withKD(OuttakeConstants.vel_kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    motorConfig = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(pivotEncoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(OuttakeConstants.mechGearRatio)
                .withSensorToMechanismRatio(1.0))
        .withSlot0(positionPIDConfigs)
        .withSlot1(velocityPIDConfigs)
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs()
                .withReverseLimitEnable(true)
                .withReverseLimitSource(OuttakeConstants.limitSwitchPort)
                .withReverseLimitRemoteSensorID(OuttakeConstants.CANdiID))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(OuttakeConstants.reverseSoftLimitAngle)
                .withForwardSoftLimitThreshold(OuttakeConstants.forwardSoftLimitAngle))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(OuttakeConstants.currentLimit));
    StatusCode motorConfigStatus = motor.getConfigurator().apply(motorConfig);

    Angle pivotDiscontinuityPoint = ((OuttakeConstants.reverseSoftLimitAngle
        .plus(OuttakeConstants.forwardSoftLimitAngle))
        .div(2))
        .plus(Rotations.of(0.5));

    encoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(pivotDiscontinuityPoint)
        .withMagnetOffset(OuttakeConstants.encoderMagnetOffset));
    StatusCode encoderStatusCode = pivotEncoder.getConfigurator().apply(encoderConfig);

    candiConfig = new CANdiConfiguration();
    StatusCode candiConfigStatus = hardstop.getConfigurator().apply(candiConfig);

    if (motorConfigStatus != StatusCode.OK || encoderStatusCode != StatusCode.OK
        || candiConfigStatus != StatusCode.OK) {
      throw new IllegalStateException("Haha ur fucked");
    }
    motorKt = motor.getMotorKT().getValue();
  }

  private boolean getHardStopValue() {
    return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  private Torque getTorque() {
    motorKt = motor.getMotorKT().getValue();
    Current torqueCurrent = motor.getTorqueCurrent().getValue();
    return (Torque) motorKt.timesDivisor(torqueCurrent);
  }

  private Angle getPosition() {
    return motor.getPosition().getValue();
  }

  // at pre prescribed position
  private boolean atSetPosition() {
    if (motor.getControlMode().getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
      return false;
    }

    return (Rotations.of(Math.abs(motor.getClosedLoopError().getValue())))
        .lte(OuttakeConstants.acceptablePositionError);
  }

  // at given position
  private boolean atPosition(Angle target) {
    Angle error = target.minus(getPosition());
    Angle absError = Radians.of(Math.abs(error.in(Radians)));
    return absError.lte(OuttakeConstants.acceptablePositionError);
  }

  private void setPosition(Angle posAngle) {
    zeroReported = false;
    if (posAngle.gt(OuttakeConstants.forwardSoftLimitAngle)) {
      posAngle = OuttakeConstants.forwardSoftLimitAngle;
    } else if (posAngle.lt(OuttakeConstants.reverseSoftLimitAngle)) {
      posAngle = OuttakeConstants.reverseSoftLimitAngle;
    }

    System.out.print("Position: ");
    System.out.println(posAngle.toLongString());
    motor.setControl(posRequest.withPosition(posAngle));
  }

  private AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  boolean zeroReported = false;

  private void setVelocity(AngularVelocity setVelocity) {
    if (getHardStopValue() && setVelocity.in(RPM) < 0) {
      setVelocity = RPM.of(0);
    } else if (getPosition().gt(OuttakeConstants.forwardSoftLimitAngle) && setVelocity.in(RPM) > 0) {
      setVelocity = RPM.of(0);
    }
    if (!setVelocity.isEquivalent(RPM.of(0))) {
      zeroReported = false;
      System.out.print("Velocity: ");
      System.out.println(setVelocity.toLongString());
    } else if (!zeroReported) {
      zeroReported = true;
      System.out.print("Velocity: ");
      System.out.println(setVelocity.toLongString());
    }
    motor.setControl(velRequest.withVelocity(setVelocity));
  }

  private void setVoltage(Voltage setVoltage) {
    if (getHardStopValue() && setVoltage.in(Volts) < 0) {
      setVoltage = Volts.of(0);
    } else if (getPosition().gt(OuttakeConstants.forwardSoftLimitAngle) && setVoltage.in(Volts) > 0) {
      setVoltage = Volts.of(0);
    }

    motor.setControl(voltageRequest.withOutput(setVoltage));
  }

  private Command goToPositionFactory(Angle position, boolean blocking) {
    Command baseCommand = this.runOnce(() -> {
      setPosition(position);
    });
    Command waitForPosition = Commands.waitUntil(() -> atSetPosition());

    if (blocking) {
      return baseCommand.andThen(waitForPosition);
    } else {
      return baseCommand;
    }
  }

  public Command goToHome(boolean blocking) {
    return goToPositionFactory(OuttakeConstants.homeAngle, blocking);
  }

  public Command goToHome() {
    return goToHome(false);
  }

  public Command goToTopBox(boolean blocking) {
    return goToPositionFactory(OuttakeConstants.topBoxAngle, blocking);
  }

  public Command goToTopBox() {
    return goToTopBox(false);
  }

  public Command goToBottomBox(boolean blocking) {
    return goToPositionFactory(OuttakeConstants.bottomBoxAngle, blocking);
  }

  public Command goToBottomBox() {
    return goToBottomBox(false);
  }

  private Command selectStepUpCommand() {
    Angle currentPos = getPosition();
    System.out.print(currentPos.toLongString());
    System.out.print(" ");
    if (currentPos.lt(OuttakeConstants.bottomBoxAngle) && !atPosition(OuttakeConstants.bottomBoxAngle)) {
      System.out.println(1);
      return goToBottomBox(true);
    } else if (currentPos.lt(OuttakeConstants.topBoxAngle) && !atPosition(OuttakeConstants.topBoxAngle)) {
      System.out.println(2);
      return goToTopBox(true);
    } else if (currentPos.lt(OuttakeConstants.homeAngle) && !atPosition(OuttakeConstants.homeAngle)) {
      System.out.println(3);
      return goToHome(true);
    } else {
      System.out.println(4);
      return goToBottomBox(true);
    }
  }

  public Command stepUp() {
    return Commands.defer(this::selectStepUpCommand, Set.of(this));
  }

  private Command selectStepDownCommand() {
    Angle currentPos = getPosition();
    System.out.print(currentPos.toLongString());
    System.out.print(" ");
    if (currentPos.gt(OuttakeConstants.homeAngle) && !atPosition(OuttakeConstants.homeAngle)) {
      System.out.println(5);
      return goToHome(true);
    } else if (currentPos.gt(OuttakeConstants.topBoxAngle) && !atPosition(OuttakeConstants.topBoxAngle)) {
      System.out.println(6);
      return goToTopBox(true);
    } else if (currentPos.gt(OuttakeConstants.bottomBoxAngle) && !atPosition(OuttakeConstants.bottomBoxAngle)) {
      System.out.println(7);
      return goToBottomBox(true);
    } else {
      System.out.println(8);
      return goToHome(true);
    }
  }

  public Command stepDown() {
    return Commands.defer(this::selectStepDownCommand, Set.of(this));
  }

  public Command VelocityControl(DoubleSupplier negativeInput, DoubleSupplier positiveInput) {
    return this.run(() -> {
      double positiveValue = positiveInput.getAsDouble();
      double negativeValue = negativeInput.getAsDouble();
      double shapedInput = ((positiveValue * positiveValue) - (negativeValue * negativeValue)); // bad name wth
      AngularVelocity desiredVelocity = OuttakeConstants.maxSafeVelocity.times(shapedInput);

      setVelocity(desiredVelocity);
    });
  }

  /*
   * No Joysticks(oops)
   * public Command JoystickPositionControl(DoubleSupplier joystickInput) {
   * return this.run(() -> {
   * Angle desiredAngle = GRTUtils.mapJoystick(joystickInput.getAsDouble(),
   * OuttakeConstants.reverseSoftLimitAngle,
   * OuttakeConstants.homeAngle);
   * if (desiredAngle.gt(OuttakeConstants.reverseSoftLimitAngle)) {
   * desiredAngle = OuttakeConstants.reverseSoftLimitAngle;
   * }
   * setPosition(desiredAngle);
   * });
   * }
   * 
   * public Command JoystickVelocityControl(DoubleSupplier joystickInput) {
   * return this.run(() -> {
   * double rawInput = joystickInput.getAsDouble();
   * double shapedInput = Math.copySign(rawInput * rawInput, rawInput);
   * 
   * AngularVelocity desiredVelocity =
   * OuttakeConstants.maxSafeVelocity.times(shapedInput);
   * if (getHardStopValue() && desiredVelocity.in(RPM) > 0) {
   * desiredVelocity = RPM.of(0);
   * }
   * setVelocity(desiredVelocity);
   * });
   * }
   * 
   * public Command JoystickVoltageControl(DoubleSupplier joystickInput) {
   * return this.run(() -> {
   * double rawInput = joystickInput.getAsDouble();
   * double shapedInput = Math.copySign(rawInput * rawInput, rawInput);
   * 
   * Voltage maxVoltage =
   * OuttakeConstants.maxSafeVelocity.div(OuttakeConstants.approximateMaxVelocity)
   * .times
   * (Volts.of(12));
   * Voltage outputVoltage = maxVoltage.times(shapedInput);
   * if (getHardStopValue() && outputVoltage.in(Volts) > 0) {
   * outputVoltage = Volts.of(0);
   * }
   * setVoltage(outputVoltage);
   * });
   * }
   */

  // private final DCMotor m_motorSim = DCMotor.getKrakenX60Foc(1);
  // private final SingleJointedArmSim m_armSim = new
  // SingleJointedArmSim(m_motorSim,
  // OuttakeConstants.mechGearRatio,
  // OuttakeConstants.armMomentOfInertia.in(KilogramSquareMeters),
  // OuttakeConstants.armLength.in(Meters),
  // OuttakeConstants.reverseSoftLimitAngle.in(Radians),
  // OuttakeConstants.forwardSoftLimitAngle.in(Radians),
  // true,
  // 0);

  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor.getSimState();
    var hardstopSim = hardstop.getSimState();
    // var cancoderSim = pivotEncoder.getSimState();

    // talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // m_armSim.setInputVoltage(motorVoltage.in(Volts));
    // m_armSim.update(0.020);

    // Angle armAngle = Radians.of(m_armSim.getAngleRads());
    // Angle motorAngle = armAngle.times(OuttakeConstants.mechGearRatio);

    // AngularVelocity armVelocity =
    // RadiansPerSecond.of(m_armSim.getVelocityRadPerSec());
    // AngularVelocity motorVelocity =
    // armVelocity.times(OuttakeConstants.mechGearRatio);

    // cancoderSim.setRawPosition(armAngle);
    // cancoderSim.setVelocity(armVelocity);

    // talonFXSim.setRawRotorPosition(motorAngle);
    // talonFXSim.setRotorVelocity(motorVelocity);

    boolean simHardstopTriggered = getPosition().lte(OuttakeConstants.reverseSoftLimitAngle);
    S1StateValue hardstopValue = simHardstopTriggered ? S1StateValue.High : S1StateValue.Low;
    hardstopSim.setS1State(hardstopValue);
    talonFXSim.setReverseLimit(simHardstopTriggered);
  }
}
