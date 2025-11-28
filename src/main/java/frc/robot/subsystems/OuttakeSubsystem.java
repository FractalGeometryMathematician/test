package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GRTUtils;
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
                .withNeutralMode(NeutralModeValue.Brake))
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(true)
                .withForwardLimitSource(OuttakeConstants.limitSwitchPort)
                .withForwardLimitRemoteSensorID(OuttakeConstants.CANdiID))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(OuttakeConstants.homeAngle))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(OuttakeConstants.currentLimit));
    StatusCode motorConfigStatus = motor.getConfigurator().apply(motorConfig);

    Angle pivotDiscontinuityPoint = ((OuttakeConstants.lowerLimitAngle.plus(OuttakeConstants.homeAngle)).div(2))
        .plus(Rotations.of(0.5));

    encoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(pivotDiscontinuityPoint)
        .withMagnetOffset(OuttakeConstants.encoderMagnetOffset));
    StatusCode encoderStatusCode = pivotEncoder.getConfigurator().apply(encoderConfig);

    if (motorConfigStatus != StatusCode.OK || encoderStatusCode != StatusCode.OK) {
      throw new IllegalStateException("Haha ur fucked");
    }
    motorKt = motor.getMotorKT().getValue();
  }

  public boolean getHardStopValue() {
    ForwardLimitValue curSwitchValue = motor.getForwardLimit().getValue();
    if (curSwitchValue == ForwardLimitValue.ClosedToGround) {
      return true;
    } else {
      return false;
    }
  }

  public Torque getTorque() {
    motorKt = motor.getMotorKT().getValue();
    Current torqueCurrent = motor.getTorqueCurrent().getValue();
    return (Torque) motorKt.timesDivisor(torqueCurrent);
  }

  public Angle getPosition() {
    return motor.getPosition().getValue();
  }

  public boolean atPosition() {
    if (motor.getControlMode().getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
      return false;
    }

    return Rotations.of(Math.abs(motor.getClosedLoopError().getValue())).lte(OuttakeConstants.acceptablePositionError);
  }

  public void setPosition(Angle posAngle) {
    motor.setControl(posRequest.withPosition(posAngle));
  }

  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  public void setVelocity(AngularVelocity setVelocity) {
    motor.setControl(velRequest.withVelocity(setVelocity));
  }

  public void setVoltage(Voltage setVoltage) {
    motor.setControl(voltageRequest.withOutput(setVoltage));
  }

  private Command goToPositionFactory(Angle position, boolean blocking) {
    Command baseCommand = this.runOnce(() -> {
      setPosition(position);
    });
    Command waitForPosition = Commands.waitUntil(() -> atPosition());

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

  public Command VelocityControl(DoubleSupplier negativeInput, DoubleSupplier positiveInput) {
    return this.run(() -> {
      double positiveValue = positiveInput.getAsDouble();
      double negativeValue = negativeInput.getAsDouble();
      double shapedInput = ((positiveValue * positiveValue) - (negativeValue * negativeValue)); // bad name wth
      AngularVelocity desiredSpeed = OuttakeConstants.maxSafeSpeed.times(shapedInput);
      if (getHardStopValue() && desiredSpeed.in(RPM) > 0) {
        desiredSpeed = RPM.of(0);
      }
      setVelocity(desiredSpeed);
    });
  }

  /*
   * No Joysticks(oops)
   * public Command JoystickPositionControl(DoubleSupplier joystickInput) {
   * return this.run(() -> {
   * Angle desiredAngle = GRTUtils.mapJoystick(joystickInput.getAsDouble(),
   * OuttakeConstants.lowerLimitAngle,
   * OuttakeConstants.homeAngle);
   * if (desiredAngle.gt(OuttakeConstants.lowerLimitAngle)) {
   * desiredAngle = OuttakeConstants.lowerLimitAngle;
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
   * AngularVelocity desiredSpeed =
   * OuttakeConstants.maxSafeSpeed.times(shapedInput);
   * if (getHardStopValue() && desiredSpeed.in(RPM) > 0) {
   * desiredSpeed = RPM.of(0);
   * }
   * setVelocity(desiredSpeed);
   * });
   * }
   * 
   * public Command JoystickVoltageControl(DoubleSupplier joystickInput) {
   * return this.run(() -> {
   * double rawInput = joystickInput.getAsDouble();
   * double shapedInput = Math.copySign(rawInput * rawInput, rawInput);
   * 
   * Voltage maxVoltage =
   * OuttakeConstants.maxSafeSpeed.div(OuttakeConstants.approximateMaxSpeed).times
   * (Volts.of(12));
   * Voltage outputVoltage = maxVoltage.times(shapedInput);
   * if (getHardStopValue() && outputVoltage.in(Volts) > 0) {
   * outputVoltage = Volts.of(0);
   * }
   * setVoltage(outputVoltage);
   * });
   * }
   */
}
