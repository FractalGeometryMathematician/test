package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {
  // Motor Kt Signal
  private StatusSignal<Per<TorqueUnit, CurrentUnit>> motorKt;

  // Motor Object (Kraken x60)
  private final TalonFX motor = new TalonFX(OuttakeConstants.motorID, "can");

  // TalonFX Control Requests (utilizing FOC)
  final PositionTorqueCurrentFOC posRequest = new PositionTorqueCurrentFOC(Rotations.of(0)).withSlot(0);
  final VelocityTorqueCurrentFOC velRequest = new VelocityTorqueCurrentFOC(RPM.of(0)).withSlot(1);
  final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Encoder Object (WCP ThroughBore Encoder Powered by CANcoder)
  private final CANcoder pivotEncoder = new CANcoder(OuttakeConstants.encoderID, "can");

  // CANdi (integrates limit switch input to CAN Bus)
  private final CANdi hardstop = new CANdi(OuttakeConstants.CANdiID, "can");

  public OuttakeSubsystem() {
    StatusCode motorConfigStatus = configureMotors();
    StatusCode encoderStatusCode = configureEncoder();
    StatusCode CANdiConfigStatus = configureCANdi();

    // Throw error if configuration fails to apply to devices
    if (motorConfigStatus != StatusCode.OK || encoderStatusCode != StatusCode.OK
        || CANdiConfigStatus != StatusCode.OK) {
      throw new IllegalStateException("Haha ur fucked");
    }

    motorKt = motor.getMotorKT();
  }

  private StatusCode configureMotors() {
    // Set PID values for position control
    final Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withKG(OuttakeConstants.pos_kG)
        .withKS(OuttakeConstants.pos_kS)
        .withKP(OuttakeConstants.pos_kP)
        .withKI(OuttakeConstants.pos_kI)
        .withKD(OuttakeConstants.pos_kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    // Set PID values for velocity control
    final Slot1Configs velocityPIDConfigs = new Slot1Configs()
        .withKG(OuttakeConstants.vel_kG)
        .withKS(OuttakeConstants.vel_kS)
        .withKP(OuttakeConstants.vel_kP)
        .withKI(OuttakeConstants.vel_kI)
        .withKD(OuttakeConstants.vel_kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        // Use CANcoder as PID feedback and set gear ratio
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(pivotEncoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(OuttakeConstants.mechGearRatio)
                .withSensorToMechanismRatio(1.0))
        // Add PID Configs
        .withSlot0(positionPIDConfigs)
        .withSlot1(velocityPIDConfigs)
        // Set braking mode and define pointing up as the positive direction
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        // Add limit switch connected to CANdi as downward hard stop
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs()
                .withReverseLimitEnable(true)
                .withReverseLimitSource(OuttakeConstants.limitSwitchPort)
                .withReverseLimitRemoteSensorID(OuttakeConstants.CANdiID))
        // Set soft limits on either end of the arm's range of movement
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(OuttakeConstants.reverseSoftLimitAngle)
                .withForwardSoftLimitThreshold(OuttakeConstants.forwardSoftLimitAngle))
        // Cap stator current
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(OuttakeConstants.currentLimit));
    return motor.getConfigurator().apply(motorConfig);
  }

  private StatusCode configureEncoder() {
    // Point at which arm would "wrap around" (CTRE docs say: mean(lowerLimit,
    // upperLimit) + 0.5)
    Angle pivotDiscontinuityPoint = ((OuttakeConstants.reverseSoftLimitAngle
        .plus(OuttakeConstants.forwardSoftLimitAngle))
        .div(2))
        .plus(Rotations.of(0.5));

    // Add discontinuity point and offset
    final CANcoderConfiguration encoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(pivotDiscontinuityPoint)
        .withMagnetOffset(OuttakeConstants.encoderMagnetOffset));
    return pivotEncoder.getConfigurator().apply(encoderConfig);
  }

  private StatusCode configureCANdi() {
    final CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    // When S1 is floating: pull input high, and is closed when pulled low
    CANdiConfig.DigitalInputs
        .withS1FloatState(S1FloatStateValue.PullHigh)
        .withS1CloseState(S1CloseStateValue.CloseWhenLow);
    return hardstop.getConfigurator().apply(CANdiConfig);
  }

  // check if hard stop limit switch is pressed
  private boolean isAtHardStop() {
    return hardstop.getS1Closed().getValue();
  }

  // check if hard stop limit switch is pressed
  private boolean isAtForwardSoftStop() {
    return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  // set the CANcoder position to arbitrary position
  private void setEncoder(Angle position) {
    pivotEncoder.setPosition(position);
  }

  // Gets the torque applied by motor shaft based on motor kT and current
  private Torque getTorque() {
    Current torqueCurrent = motor.getTorqueCurrent().getValue();
    return (Torque) motorKt.getValue().timesDivisor(torqueCurrent);
  }

  // Gets current arm position from the CANcoder
  private Angle getPosition() {
    return motor.getPosition().getValue();
  }

  // Checks if arm is at position set by PID control with a tolerance
  private boolean atSetPosition() {
    // if not in position control return false
    if (motor.getControlMode().getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
      return false;
    }

    // checks if closed loop error is within tolerance
    return (Rotations.of(Math.abs(motor.getClosedLoopError().getValue())))
        .lte(OuttakeConstants.acceptablePositionError);
  }

  // Checks if arm is at given position
  private boolean atPosition(Angle target) {
    // finds error between position and target and its absolute value
    Angle error = target.minus(getPosition());
    Angle absError = Radians.of(Math.abs(error.in(Radians)));

    // checks if difference is within tolerance
    return absError.lte(OuttakeConstants.acceptablePositionError);
  }

  // Uses PID to command arm to set position
  private void setPosition(Angle posAngle) {
    zeroReported = false;
    // If the desired position is out of the range of the arm, clamp the set
    // position to relevant extreme
    if (posAngle.gt(OuttakeConstants.forwardSoftLimitAngle)) {
      posAngle = OuttakeConstants.forwardSoftLimitAngle;
    } else if (posAngle.lt(OuttakeConstants.reverseSoftLimitAngle)) {
      posAngle = OuttakeConstants.reverseSoftLimitAngle;
    }

    System.out.print("Position: ");
    System.out.println(posAngle.toLongString());
    // Set motor control
    motor.setControl(posRequest.withPosition(posAngle));
  }

  // Get arm velocity from CANcoder
  private AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  boolean zeroReported = false;

  // Set the motor velocity with PID
  private void setVelocity(AngularVelocity setVelocity) {
    // If at hard or soft stop, set velocity to zero if going in direction of stop
    if (isAtHardStop() && setVelocity.in(RPM) < 0) {
      setVelocity = RPM.of(0);
    } else if (isAtForwardSoftStop() && setVelocity.in(RPM) > 0) {
      setVelocity = RPM.of(0);
    }
    // Print statement logging (will remove)
    if (!setVelocity.isEquivalent(RPM.of(0))) {
      zeroReported = false;
      System.out.print("Velocity: ");
      System.out.println(setVelocity.toLongString());
    } else if (!zeroReported) {
      zeroReported = true;
      System.out.print("Velocity: ");
      System.out.println(setVelocity.toLongString());
    }

    // Control motor with velocity PID
    motor.setControl(velRequest.withVelocity(setVelocity));
  }

  // Directly set the motor voltage (preferred over DutyCycle for repeatability)
  private void setVoltage(Voltage setVoltage) {
    // If at hard or soft stop, do not move if desired motion is into the stop
    if (isAtHardStop() && setVoltage.in(Volts) < 0) {
      setVoltage = Volts.of(0);
    } else if (isAtForwardSoftStop() && setVoltage.in(Volts) > 0) {
      setVoltage = Volts.of(0);
    }

    // Control motor voltage
    motor.setControl(voltageRequest.withOutput(setVoltage));
  }

  // Returns a command that goes to specific point and waits if blocking is set to
  // true
  private Command goToPositionFactory(Angle position, boolean blocking) {
    // command to go to position
    Command baseCommand = this.runOnce(() -> {
      setPosition(position);
    });
    // wait for motor to go to position
    Command waitForPosition = Commands.waitUntil(() -> atSetPosition());

    // return a blocking command if blocking is true
    if (blocking) {
      return baseCommand.andThen(waitForPosition);
    } else {
      return baseCommand;
    }
  }

  // Command that sets arm to home position and waits if blocking is true
  public Command goToHome(boolean blocking) {
    return goToPositionFactory(OuttakeConstants.homeAngle, blocking);
  }

  // Default command that doesn't block
  public Command goToHome() {
    return goToHome(false);
  }

  // Command that sets arm to the top-box scoring position and waits if blocking
  // is true
  public Command goToTopBox(boolean blocking) {
    return goToPositionFactory(OuttakeConstants.topBoxAngle, blocking);
  }

  // Default command that sends arm to top-box position without blocking
  public Command goToTopBox() {
    return goToTopBox(false);
  }

  // Command that sets arm to the bottom-box scoring position and waits if
  // blocking is true
  public Command goToBottomBox(boolean blocking) {
    return goToPositionFactory(OuttakeConstants.bottomBoxAngle, blocking);
  }

  // Default command that sends arm to bottom-box position without blocking
  public Command goToBottomBox() {
    return goToBottomBox(false);
  }

  // Returns command to go the next highest position from the current position
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

  // Steps up the command arm (by default, wpilib would precompile the stepup
  // command based on initial position but defer fixes it)
  public Command stepUp() {
    return Commands.defer(this::selectStepUpCommand, Set.of(this));
  }

  // Returns command to go the next lowest position from the current position
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

  // Steps down the command arm (by default, wpilib would precompile the stepdown
  // command based on initial position but defer fixes it)
  public Command stepDown() {
    return Commands.defer(this::selectStepDownCommand, Set.of(this));
  }

  // Controls motor velocity with two inputs, one which sets negative velocity and
  // one sets positive velocity
  public Command VelocityControl(DoubleSupplier negativeInput, DoubleSupplier positiveInput) {
    return this.run(() -> {
      double positiveValue = positiveInput.getAsDouble();
      double negativeValue = negativeInput.getAsDouble();
      // Squares the two inputs to make the lower values more sensitive and combines
      // the two values
      double shapedInput = ((positiveValue * positiveValue) - (negativeValue * negativeValue)); // bad name wth
      // find the velocity based on the shaped input and the maximum safe velocity
      AngularVelocity desiredVelocity = OuttakeConstants.maxSafeVelocity.times(shapedInput);

      // sets the desired velocity of the motor
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
   * if (isAtHardStop() && desiredVelocity.in(RPM) > 0) {
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
   * if (isAtHardStop() && outputVoltage.in(Volts) > 0) {
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
    S1StateValue hardstopValue = simHardstopTriggered ? S1StateValue.Low : S1StateValue.High;
    hardstopSim.setS1State(hardstopValue);
    talonFXSim.setReverseLimit(simHardstopTriggered);
  }
}
