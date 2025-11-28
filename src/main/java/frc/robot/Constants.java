// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

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

  public static final class OuttakeConstants {
    // CAN IDs
    public static final int motorID = 0;
    public static final int encoderID = 0;
    public static final int CANdiID = 0;

    public static final ReverseLimitSourceValue limitSwitchPort = ReverseLimitSourceValue.RemoteCANdiS1;

    // PID Constants
    public static final float pos_kG = 0.0f;
    public static final float pos_kS = 0.0f;
    public static final float pos_kP = 0.0f;
    public static final float pos_kI = 0.0f;
    public static final float pos_kD = 0.0f;

    public static final float vel_kG = 0.0f;
    public static final float vel_kS = 0.0f;
    public static final float vel_kP = 0.0f;
    public static final float vel_kI = 0.0f;
    public static final float vel_kD = 0.0f;

    // Positions
    public static final Angle homeAngle = Degrees.of(90);
    public static final Angle topBoxAngle = Degrees.of(-20);
    public static final Angle bottomBoxAngle = Degrees.of(-60);
    public static final Angle reverseSoftLimitAngle = Degrees.of(-80);
    public static final Angle forwardSoftLimitAngle = homeAngle;

    // Hardware Constants
    public static final Angle encoderMagnetOffset = Degrees.of(3);
    public static final float mechGearRatio = 8.0f;
    public static final AngularVelocity approximateMaxSpeed = RPM.of(5000 / mechGearRatio);

    // Settings
    public static final Current currentLimit = Amps.of(120);
    public static final AngularVelocity maxSafeSpeed = RPM.of(120);
    public static final Angle acceptablePositionError = Degrees.of(3);
    public static final AngularVelocity acceptableVelocityError = RotationsPerSecond.of(1);
  }
}
