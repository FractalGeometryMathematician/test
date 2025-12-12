package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;

import java.util.EnumSet;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GRTUtils;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {
  // Motor Kt (Nm/A)
  private Per<TorqueUnit, CurrentUnit> motorKt;

  // TalonFX Motor Object (Kraken x60)
  private final TalonFX motor = new TalonFX(OuttakeConstants.motorID, "can");

  // TalonFX Control Requests (utilizing FOC)
  final PositionTorqueCurrentFOC posRequest = new PositionTorqueCurrentFOC(Rotations.of(0)).withSlot(0);
  final VelocityTorqueCurrentFOC velRequest = new VelocityTorqueCurrentFOC(RPM.of(0)).withSlot(1);
  final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Encoder Object (WCP ThroughBore Encoder Powered by CANcoder)
  private final CANcoder pivotEncoder = new CANcoder(OuttakeConstants.encoderID, "can");

  // CANdi Objects (integrates limit switch input to CAN Bus)
  private final CANdi hardstop = new CANdi(OuttakeConstants.CANdiID, "can");

  // Mechanism2d visualization
  private final Mechanism2d mechanism;
  private final MechanismRoot2d pivotRoot;
  private final MechanismLigament2d pivotArm;

  public OuttakeSubsystem() {
    StatusCode motorConfigStatus = configureMotors();
    StatusCode encoderStatusCode = configureEncoder();
    StatusCode CANdiConfigStatus = configureCANdi();

    // Throw error if configuration fails to apply to devices
    if (motorConfigStatus != StatusCode.OK || encoderStatusCode != StatusCode.OK
        || CANdiConfigStatus != StatusCode.OK) {
      throw new IllegalStateException("Haha ur fucked");
    }

    DataLogManager.start();

    // Create Mechanism2d visualization (3m x 3m canvas)
    mechanism = new Mechanism2d(3, 3);
    // Root at bottom center (1.5m from left, 0.5m from bottom)
    pivotRoot = mechanism.getRoot("pivot", 1.5, 1.5);
    // Arm ligament - scale up by 4x for better visibility (visual only, doesn't
    // affect physics)
    double visualLength = OuttakeConstants.armLength.in(Meters) * 4.0;
    pivotArm = pivotRoot.append(
        new MechanismLigament2d("arm", visualLength, 0, 6, new Color8Bit(Color.kOrange)));
    // Publish to SmartDashboard
    SmartDashboard.putData("Simulation/Pivot", mechanism);

    updateLogging();

    // create a trigger based on hardstop limit switch and reset encoder when limit
    // switch triggered
    Trigger hardStopTrigger = new Trigger(this::isAtHardStop);
    hardStopTrigger.onTrue(this.runOnce(() -> setEncoder(OuttakeConstants.reverseSoftLimitAngle)));

    // update motor Kt value
    motorKt = motor.getMotorKT().getValue();
    configNT();
  }

  public void configPID(double p, double i, double d, double ff) {

    Slot0Configs slot0Configs = new Slot0Configs(); // used to store and update PID values
    slot0Configs.kP = p;
    slot0Configs.kI = i;
    slot0Configs.kD = d;
    slot0Configs.kS = ff;
    slot0Configs.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    slot0Configs.withGravityType(GravityTypeValue.Arm_Cosine);

    motor.getConfigurator().apply(slot0Configs);
  }

  private void configNT() {
    NetworkTableInstance.getDefault().getTable("outtakeNTDebug")
        .getEntry("PIDF")
        .setDoubleArray(
            new double[] {
                OuttakeConstants.pos_kP,
                OuttakeConstants.pos_kI,
                OuttakeConstants.pos_kD,
                OuttakeConstants.kS
            });
    NetworkTableInstance.getDefault().getTable("outtakeNTDebug").addListener(
        "PIDF",
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (table, key, event) -> {
          double[] pidf = event.valueData.value.getDoubleArray();
          configPID(pidf[0], pidf[1], pidf[2], pidf[3]);
        });

  }

  private StatusCode configureMotors() {
    // Set PID values for position control
    final Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withKG(OuttakeConstants.kG)
        .withKS(OuttakeConstants.kS)
        .withKP(OuttakeConstants.pos_kP)
        .withKI(OuttakeConstants.pos_kI)
        .withKD(OuttakeConstants.pos_kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    // Set PID values for velocity control
    final Slot1Configs velocityPIDConfigs = new Slot1Configs()
        .withKG(OuttakeConstants.kG)
        .withKS(OuttakeConstants.kS)
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
                .withInverted(InvertedValue.Clockwise_Positive)
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
        .withMagnetOffset(OuttakeConstants.encoderMagnetOffset)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
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

  private void updateLogging() {
    SmartDashboard.putNumber("Motor/Supply Voltage", GRTUtils.round(getSupplyVoltage().in(Volts), 2));
    SmartDashboard.putNumber("Motor/Applied Voltage", GRTUtils.round(getMotorVoltage().in(Volts), 2));
    SmartDashboard.putNumber("Motor/Supply Current", GRTUtils.round(getSupplyCurrent().in(Amps), 2));
    SmartDashboard.putNumber("Motor/Torque Current", GRTUtils.round(getTorqueCurrent().in(Amps), 2));
    SmartDashboard.putNumber("Motor/Applied Torque", GRTUtils.round(getTorque().in(NewtonMeters), 2));

    SmartDashboard.putNumber("Motor/Velocity", GRTUtils.round(getVelocity().in(RPM), 2));
    SmartDashboard.putNumber("Motor/Absolute Position", GRTUtils.round(getPosition().in(Degrees), 2));

    SmartDashboard.putString("PID/Control Mode", motor.getControlMode().toString());
    SmartDashboard.putNumber("PID/Setpoint", GRTUtils.round(motor.getClosedLoopReference().getValueAsDouble(), 2)); // RPS
    SmartDashboard.putNumber("PID/Error", GRTUtils.round(motor.getClosedLoopError().getValueAsDouble(), 2));
    SmartDashboard.putString("PID/Unit", motor.getClosedLoopReference().getUnits());
    SmartDashboard.putBoolean("PID/atPosition", atSetPosition());

    pivotArm.setAngle(getPosition().in(Degrees));
  }

  // check if hard stop limit switch is pressed
  public boolean isAtHardStop() {
    return hardstop.getS1Closed().getValue();
  }

  // check if hard stop limit switch is pressed
  public boolean isAtForwardSoftStop() {
    return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  // set the CANcoder position to arbitrary position
  public void setEncoder(Angle position) {
    pivotEncoder.setPosition(position);
  }

  // Gets the torque applied by motor shaft based on motor kT and current
  public Torque getTorque() {
    motorKt = motor.getMotorKT().getValue();
    Current torqueCurrent = motor.getTorqueCurrent().getValue();
    return (Torque) motorKt.timesDivisor(torqueCurrent);
  }

  // Gets current arm position from the CANcoder
  public Angle getPosition() {
    return motor.getPosition().getValue();
  }

  // Gets current voltage supplied to motor
  public Voltage getSupplyVoltage() {
    return motor.getSupplyVoltage().getValue();
  }

  // Gets the applied motor voltage
  public Voltage getMotorVoltage() {
    return motor.getMotorVoltage().getValue();
  }

  // Gets the current supplied to the motor
  public Current getSupplyCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  // Gets the applied motor current
  public Current getTorqueCurrent() {
    return motor.getTorqueCurrent().getValue();
  }

  // Checks if arm is at position set by PID control with a tolerance
  public boolean atSetPosition() {
    // if not in position control return false
    if (motor.getControlMode().getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
      return false;
    }

    // checks if closed loop error is within tolerance
    return (Rotations.of(Math.abs(motor.getClosedLoopError().getValue())))
        .lte(OuttakeConstants.acceptablePositionError);
  }

  // Checks if arm is at given position
  public boolean atPosition(Angle target) {
    // finds error between position and target and its absolute value
    Angle error = target.minus(getPosition());
    Angle absError = Radians.of(Math.abs(error.in(Radians)));

    // checks if difference is within tolerance
    return absError.lte(OuttakeConstants.acceptablePositionError);
  }

  // Uses PID to command arm to set position
  public void setPosition(Angle posAngle) {
    // If the desired position is out of the range of the arm, clamp the set
    // position to relevant extreme
    if (posAngle.gt(OuttakeConstants.forwardSoftLimitAngle)) {
      posAngle = OuttakeConstants.forwardSoftLimitAngle;
    } else if (posAngle.lt(OuttakeConstants.reverseSoftLimitAngle)) {
      posAngle = OuttakeConstants.reverseSoftLimitAngle;
    }

    DataLogManager.log("Going to position: " + posAngle.toLongString());
    // Set motor control
    motor.setControl(posRequest.withPosition(posAngle));
  }

  // Get arm velocity from CANcoder
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  // Set the motor velocity with PID
  public void setVelocity(AngularVelocity setVelocity) {
    // If at hard or soft stop, set velocity to zero if going in direction of stop
    if (isAtHardStop() && setVelocity.in(RPM) < 0) {
      setVelocity = RPM.of(0);
    } else if (isAtForwardSoftStop() && setVelocity.in(RPM) > 0) {
      setVelocity = RPM.of(0);
    }

    DataLogManager.log(setVelocity.toLongString());

    // Control motor with velocity PID
    motor.setControl(velRequest.withVelocity(setVelocity));
  }

  // Directly set the motor voltage (preferred over DutyCycle for repeatability)
  public void setVoltage(Voltage setVoltage) {
    // If at hard or soft stop, do not move if desired motion is into the stop
    if (isAtHardStop() && setVoltage.in(Volts) < 0) {
      setVoltage = Volts.of(0);
    } else if (isAtForwardSoftStop() && setVoltage.in(Volts) > 0) {
      setVoltage = Volts.of(0);
    }

    // Control motor voltage
    motor.setControl(voltageRequest.withOutput(setVoltage));
  }

  public void setVoltage(double setValue) {
    // If at hard or soft stop, do not move if desired motion is into the stop
    if (isAtHardStop() && setValue < 0) {
      setValue = 0;
    } else if (isAtForwardSoftStop() && setValue > 0) {
      setValue = 0;
    }

    Voltage setVoltage = Volts.of(12 * setValue);
    DataLogManager.log(setVoltage.toLongString());

    // Control motor voltage
    motor.setControl(voltageRequest.withOutput(setVoltage));
  }

  public void setDutyCycle(double setPercent) {
    if (isAtHardStop() && setPercent < 0) {
      setPercent = 0;
    } else if (isAtForwardSoftStop() && setPercent > 0) {
      setPercent = 0;
    }

    motor.setControl(dutyCycleRequest.withOutput(setPercent));
  }

  // Returns a command that goes to specific point and waits if blocking is set to
  // true
  private Command goToPositionFactory(Angle position, boolean blocking) {
    // command to go to position
    Command baseCommand = this.runOnce(() -> {
      setPosition(position);
    });
    // wait for motor to go to position
    Command waitForPosition = Commands.waitUntil(() -> atSetPosition()).withTimeout(Seconds.of(5));

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
    DataLogManager.log(currentPos.in(Degrees) + " degrees");
    if (currentPos.lt(OuttakeConstants.topBoxAngle) && !atPosition(OuttakeConstants.topBoxAngle)) {
      return goToTopBox(true);
    } else if (currentPos.lt(OuttakeConstants.homeAngle) && !atPosition(OuttakeConstants.homeAngle)) {
      return goToHome(true);
    } else {
      return goToBottomBox(true);
    }
  }

  // Steps up the command arm (by default, wpilib would precompile the stepup
  // command based on initial position but defer fixes it)
  public Command stepUp() {
    return this.defer(this::selectStepUpCommand);
  }

  // Returns command to go the next lowest position from the current position
  private Command selectStepDownCommand() {
    Angle currentPos = getPosition();
    DataLogManager.log(currentPos.in(Degrees) + " degrees");
    if (currentPos.gt(OuttakeConstants.topBoxAngle) && !atPosition(OuttakeConstants.topBoxAngle)) {
      return goToTopBox(true);
    } else if (currentPos.gt(OuttakeConstants.bottomBoxAngle) && !atPosition(OuttakeConstants.bottomBoxAngle)) {
      return goToBottomBox(true);
    } else {
      return goToHome(true);
    }
  }

  // Steps down the command arm (by default, wpilib would precompile the stepdown
  // command based on initial position but defer fixes it)
  public Command stepDown() {
    return this.defer(this::selectStepDownCommand);
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

  // Controls motor voltage with two inputs, one which sets negative voltage and
  // one sets positive voltage
  public Command VoltageControl(DoubleSupplier negativeInput, DoubleSupplier positiveInput) {
    return this.run(() -> {

      // divides by ten to decrease max speed
      double positiveValue = positiveInput.getAsDouble();
      double negativeValue = negativeInput.getAsDouble();
      // Squares the two inputs to make the lower values more sensitive and combines
      // the two values
      double shapedInput = ((positiveValue * positiveValue) - (negativeValue * negativeValue)); // bad name wth

      DataLogManager.log(shapedInput + " %");
      // sets the desired voltage of the motor
      setVoltage(shapedInput);
    });
  }

  public Command valueControl(DoubleSupplier negativeInput, DoubleSupplier positiveInput) {
    return this.run(() -> {

      // divides by ten to decrease max speed
      double positiveValue = positiveInput.getAsDouble();
      double negativeValue = negativeInput.getAsDouble();
      // Squares the two inputs to make the lower values more sensitive and combines
      // the two values
      double shapedInput = ((positiveValue * positiveValue) - (negativeValue * negativeValue)) / 10; // bad name wth

      DataLogManager.log(shapedInput + " %");
      // sets the desired voltage of the motor
      setDutyCycle(shapedInput);
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

  private final DCMotor m_motorSim = DCMotor.getKrakenX60Foc(1);
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(m_motorSim,
      OuttakeConstants.mechGearRatio,
      OuttakeConstants.armMomentOfInertia.in(KilogramSquareMeters),
      OuttakeConstants.armLength.in(Meters),
      OuttakeConstants.reverseSoftLimitAngle.in(Radians),
      OuttakeConstants.forwardSoftLimitAngle.in(Radians),
      true,
      0);

  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor.getSimState();
    var hardstopSim = hardstop.getSimState();
    var cancoderSim = pivotEncoder.getSimState();

    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    m_armSim.setInputVoltage(motorVoltage.in(Volts));
    m_armSim.update(0.020);

    Angle armAngle = Radians.of(m_armSim.getAngleRads());
    Angle motorAngle = armAngle.times(OuttakeConstants.mechGearRatio);

    AngularVelocity armVelocity = RadiansPerSecond.of(m_armSim.getVelocityRadPerSec());
    AngularVelocity motorVelocity = armVelocity.times(OuttakeConstants.mechGearRatio);

    cancoderSim.setRawPosition(armAngle);
    cancoderSim.setVelocity(armVelocity);

    talonFXSim.setRawRotorPosition(motorAngle);
    talonFXSim.setRotorVelocity(motorVelocity);

    boolean simHardstopTriggered = getPosition().lte(OuttakeConstants.reverseSoftLimitAngle);
    S1StateValue hardstopValue = simHardstopTriggered ? S1StateValue.Low : S1StateValue.High;
    hardstopSim.setS1State(hardstopValue);
    talonFXSim.setReverseLimit(simHardstopTriggered);

    updateLogging();
  }

  @Override
  public void periodic() {
    updateLogging();
  }
}
