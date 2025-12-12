// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {
    // CAN IDs
    public static final int leftLeaderID = 3;
    public static final int leftFollowerID =10;
    public static final int rightLeaderID = 7;
    public static final int rightFollowerID = 9;

    // PID Constants
    public static final float kP = 0.0f;
    public static final float kI = 0.0f;
    public static final float kD = 0.0f;
    public static final float kV = 473; // RPM / V
    public static final float kS = 0.0f; // V

    public static final AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(150);
    public static final AngularVelocity maxVelocity = RPM.of(3000);
    public static final double allowedError = 0.0; // lowk idk the units
    public static final AngularVelocity maxMotorVelocity = RPM.of(3000);

    public static final Current currentLimit = Amps.of(50);
    public static final Time closedLoopRampRate = Seconds.of(5);
    public static final Time openLoopRampRate = Seconds.of(5);
  }

  public static final class PivotConstants {

    // Motors
    public static final int PIVOT_MOTOR_LEFT_ID = 4;
    public static final int PIVOT_MOTOR_RIGHT_ID = 2;
  
    // PID constants for TorqueCurrentFOC with 14:1 gear ratio
    public static final double PIVOT_P = 0.1;  // Proportional gain (torque in amps per rotation error)
    public static final double PIVOT_I = 0.0;  // Integral
    public static final double PIVOT_D = 0.0;  // Derivative
    public static final double PIVOT_KG = 0.0;  // Gravity feedforward (torque in amps to counteract gravity)
    public static final double PIVOT_KS = 0.0;  // Static friction feedforward (torque in amps to overcome static friction)
  
    public static final int ENCODER_ID = 0;
  
    // Speed and current limits
    public static final double PIVOT_MANUAL_SPEED = 0.06;  // Manual control speed (0.0 to 1.0)
    public static final double PIVOT_MAX_CURRENT = 80.0;   // Maximum torque current (amps)
    public static final double PIVOT_CURRENT_LIMIT = 80.0; // Stator current limit (amps)
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0; // Supply current limit (amps)
  
    // Angle limits (in rotations - relative encoder range)
    public static final double MIN_ANGLE = 0.0;   // Minimum position / stowed
    public static final double MAX_ANGLE = 0.14;  // Maximum position / fully raised
  
    public static final double GEAR_RATIO = 14; // 14 motor rotations = 1 output rotation
  
    // Preset positions (in rotations)
    public static final double MID_POSITION = 0.07;  // Mid position (halfway between 0 and 0.14)
    public static final double POSITION_TOLERANCE = 0.01;  // Tolerance in rotations
  }
  public static final class PneumaticsConstants {
    public static final int pneumaticForwardChannel = 1;   // Forward solenoid channel on PCM (0-7)
    public static final int pneumaticReverseChannel = 2;   // Reverse solenoid channel on PCM (0-7)
    public static final int pneumaticCANId = 10;           // PCM CAN ID (commonly 10-19)
  }
  
  public static final class ServoConstants {
    public static final int SERVO_ID = 9;  // PWM channel (0-9), changed from 0 to avoid DIO conflict
  
    // Servo positions: 0° = 0.0, 120° = 0.67 (assuming 180° servo range)
    public static final double HOME_POSITION = 0.0;  // 0 degrees - boot position
    public static final double OPEN_POSITION = 0.67; // 120 degrees counterclockwise
  
    public static final double MAX_POSITION = 1.0;
    public static final double MIN_POSITION = 0.0;
    public static final double INTAKE_TOLERANCE = 0.05;
  
    public static final double INTAKE_SPEED = 0.02;
  
  }
    
}
