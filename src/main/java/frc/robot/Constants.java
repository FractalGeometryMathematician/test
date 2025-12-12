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
    public static final int leftLeaderID = 12;
    public static final int leftFollowerID = 7;
    public static final int rightLeaderID = 11;
    public static final int rightFollowerID = 8;

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
    public static final Time closedLoopRampRate = Seconds.of(0.05);
    public static final Time openLoopRampRate = Seconds.of(0.1);
  }

  public static final class OuttakeConstants {
    // CAN IDs
    public static final int motorID = 2;
    public static final int encoderID = 0;
    public static final int CANdiID = 3;

    public static final ReverseLimitSourceValue limitSwitchPort = ReverseLimitSourceValue.RemoteCANdiS2;

    // PID Constants
    public static final float kG = 0f;
    public static final float kS = 0f;

    // Designed from arm inertia (0.0229 kg m^2), 8:1 gearing, and Kraken torque
    // constant (9.37 N m / 483 A). Targets ~0.5 s settling with ~1% overshoot.
    public static final float pos_kP = 90f;
    public static final float pos_kI = 0.1f;
    public static final float pos_kD = 15f;

    // Inner velocity loop treated as torque-current driven integrator. Tuned for
    // ~0.1 s response to 60 rpm step with minimal overshoot in SingleJointedArmSim.
    public static final float vel_kP = 25f;
    public static final float vel_kI = 40f;
    public static final float vel_kD = 0f;

    // Positions
    public static final Angle homeAngle = Rotations.of(-0.16);
    public static final Angle topBoxAngle = Rotations.of(0.1);
    public static final Angle bottomBoxAngle = Rotations.of(-0.1);
    public static final Angle reverseSoftLimitAngle = Rotations.of(-0.1);
    public static final Angle forwardSoftLimitAngle = Rotations.of(0.1);

    // Hardware Constants
    public static final Angle encoderMagnetOffset = Rotations.of(-0.12);
    public static final float mechGearRatio = 8.0f;
    public static final AngularVelocity approximateMaxVelocity = RPM.of(5000 / mechGearRatio);
    public static final MomentOfInertia armMomentOfInertia = KilogramSquareMeters.of(0.0228579824); // Calculated from
                                                                                                    // Onshape
    public static final Distance armLength = Inches.of(16.5);

    // Settings
    public static final Current currentLimit = Amps.of(120);
    public static final AngularVelocity maxSafeVelocity = RPM.of(30);
    public static final Angle acceptablePositionError = Degrees.of(3);
    public static final AngularVelocity acceptableVelocityError = RotationsPerSecond.of(1);
  }

  public static final class MusicConstants {
    // Files are deployed to src/main/deploy
    public static final String[] songFiles = new String[] {
        "nyanCat.chrp",
        "undertale.chrp",
        "mii.chrp",
        "fromTheStart.chrp"
    };
  }
}
