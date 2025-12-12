// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  // Kraken motors
  private final TalonFX leftKraken = new TalonFX(PivotConstants.PIVOT_MOTOR_LEFT_ID, "can");
  private final TalonFX rightKraken = new TalonFX(PivotConstants.PIVOT_MOTOR_RIGHT_ID, "can");

  // Encoder
  private final CANcoder canCoder = new CANcoder(PivotConstants.ENCODER_ID);

  // Duty cycle (percent output) control
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

  //Position based torque current FOC control with PID
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);

  public PivotSubsystem() {
    configMotors();
    configEncoder();
  }

  /**
   * Configures both pivot motors with PID, brake mode, & follower setup.
   * The left motor is the leader, and the right motor follows AFTER being inversed because they are
   * directly positioned opposite to each other.
   * Uses CANcoder as feedback source for improved accuracy.
   */
  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Config for TorqueCurrentFOC position control with gravity compensation
    config.Slot0.kP = PivotConstants.PIVOT_P;
    config.Slot0.kI = PivotConstants.PIVOT_I;
    config.Slot0.kD = PivotConstants.PIVOT_D;
    config.Slot0.kG = PivotConstants.PIVOT_KG;
    config.Slot0.kS = PivotConstants.PIVOT_KS;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Use CANcoder as feedback source
    config.Feedback.FeedbackRemoteSensorID = PivotConstants.ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.RotorToSensorRatio = PivotConstants.GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = 1.0;

    // Torque current limits
    config.TorqueCurrent.PeakForwardTorqueCurrent = PivotConstants.PIVOT_MAX_CURRENT;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -PivotConstants.PIVOT_MAX_CURRENT;
    config.CurrentLimits.StatorCurrentLimit = PivotConstants.PIVOT_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = PivotConstants.PIVOT_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Software soft limits DISABLED
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // Break mode holds position when stopped
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Config left motor with counter-clockwise positive
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftKraken.getConfigurator().apply(config);

    // Configure right motor opposite of left
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightKraken.getConfigurator().apply(config);

    // Right motor follows the left motor
    rightKraken.setControl(new Follower(PivotConstants.PIVOT_MOTOR_LEFT_ID, true));
  }

  //  Config encoder with discontinuity point for proper wrapping
  private void configEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    // Calculate discontinuity point: mean of limits + 0.5 rotations
    double discontinuityPoint = ((PivotConstants.MIN_ANGLE + PivotConstants.MAX_ANGLE) / 2.0) + 0.5;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;

    canCoder.getConfigurator().apply(config);
  }

   /**
   * Gets current encoder position in rotations
   * @return Current position in rotations
   */
  public double getPosition() {
    return canCoder.getPosition().getValueAsDouble();
  }


  /**
   * @return Absolute encoder position in rotations (0.0 to 1.0)
   */
  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }


  public void setPosition(double rotations) {
    // No position clamping - full range of motion

    // Encoder is on the pivot point (not affected by gear ratio)
    // Motor must rotate more due to gear ratio (motor_rotations = encoder_rotations * gear_ratio)
    double motorRotations = rotations * PivotConstants.GEAR_RATIO;

    leftKraken.setControl(positionControl.withPosition(motorRotations));
  }

 /**
   * Manually controls the pivot at a specific speed
   * No soft limits - full manual control
   *
   * @param speed Desired speed from -1.0 to 1.0
   */
  public void setManualSpeed(double speed) {
    // Debug logging
    SmartDashboard.putNumber("Pivot/Manual Speed Input", speed);
    SmartDashboard.putNumber("Pivot/Current Position", getPosition());
    SmartDashboard.putNumber("Pivot/Motor Voltage", leftKraken.getMotorVoltage().getValueAsDouble());

    // Command duty cycle to leader motor (leftKraken) - no limits
    leftKraken.setControl(dutyCycleControl.withOutput(speed));
  }

   // Set motor output to zero & stop all movement
  public void stop() {
    leftKraken.setControl(dutyCycleControl.withOutput(0));
  }

  //  Resets the encoder position to zero.
  public void zeroEncoder() {
    canCoder.setPosition(0.0);
  }
}