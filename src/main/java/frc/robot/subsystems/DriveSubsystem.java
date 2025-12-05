package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // Spark Max Motor Objects (Rev NEOs)
    private final SparkMax leftLeader = new SparkMax(DriveConstants.leftLeaderID, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(DriveConstants.leftFollowerID, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(DriveConstants.rightLeaderID, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(DriveConstants.rightFollowerID, MotorType.kBrushless);

    // Relative encoders from leader motors
    private final RelativeEncoder leftLeaderEncoder;
    private final RelativeEncoder rightLeaderEncoder;

    // PID controllers for leader motors
    private SparkClosedLoopController leftLeaderController;
    private SparkClosedLoopController rightLeaderController;

    // WPILib method to control a diff drive
    private DifferentialDrive robotDriveController;

    // Calculates arbitrary feedforward for PID controls
    private SimpleMotorFeedforward velocityFeedforward = new SimpleMotorFeedforward(DriveConstants.kS,
            1 / DriveConstants.kV);

    // Custom unit used by SparkMax Library for angular acceleration
    private final AngularAccelerationUnit RPMPerSecond = RPM.per(Second);

    public DriveSubsystem() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        globalConfig
                // Limit current going to the motor, using "smart" control to avoid current
                // spikes
                .smartCurrentLimit((int) DriveConstants.currentLimit.in(Amps))
                // Brake when 0 velocity is set
                .idleMode(IdleMode.kBrake)
                // Set PID and Max Motion constants
                .closedLoopRampRate(DriveConstants.closedLoopRampRate.in(Seconds))
                .openLoopRampRate(DriveConstants.openLoopRampRate.in(Seconds)).closedLoop
                .pid(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD).maxMotion
                .maxAcceleration(DriveConstants.maxAcceleration.in(RPMPerSecond))
                .maxVelocity(DriveConstants.maxVelocity.in(RPM))
                .allowedClosedLoopError(DriveConstants.allowedError);

        // Create configs that invert and setup followers as appropriate
        rightLeaderConfig
                .apply(globalConfig)
                .inverted(true);
        leftFollowerConfig
                .apply(globalConfig)
                .follow(leftLeader);
        rightFollowerConfig
                .apply(globalConfig)
                .follow(rightLeader);

        // Apply configs
        leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get PID controller from leader motors
        leftLeaderController = leftLeader.getClosedLoopController();
        rightLeaderController = rightLeader.getClosedLoopController();

        // Get encoder object references from leader motors
        leftLeaderEncoder = leftLeader.getEncoder();
        rightLeaderEncoder = rightLeader.getEncoder();

        // Init the differential drive object that drive motor with the lambda functions
        robotDriveController = new DifferentialDrive(
                (leftOutput) -> setLeftVelocity(DriveConstants.maxMotorVelocity.times(leftOutput)),
                (rightOutput) -> setRightVelocity(DriveConstants.maxMotorVelocity.times(rightOutput)));
    }

    // Sets the left motor velocity with PID control
    private void setLeftVelocity(AngularVelocity desiredVelocity) {
        // Calculate feed forward with WPI Lib method
        Voltage arbFF = Volts.of(velocityFeedforward.calculate(desiredVelocity.in(RPM)));

        // Drive left motors with Max Motion
        leftLeaderController.setReference(desiredVelocity.in(RPM), ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, arbFF.in(Volts));
    }

    // Sets the right motor velocity with PID control
    private void setRightVelocity(AngularVelocity desiredVelocity) {
        // Calculate feed forward with WPI Lib method
        Voltage arbFF = Volts.of(velocityFeedforward.calculate(desiredVelocity.in(RPM)));

        // Drive right motors with Max Motion
        rightLeaderController.setReference(desiredVelocity.in(RPM), ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, arbFF.in(Volts));
    }

    // Gets the left velocity with left leader encoders
    public double getLeftVelocityRPM() {
        return leftLeaderEncoder.getVelocity();
    }

    // Gets the right velocity with left leader encoders
    public double getRightVelocityRPM() {
        return rightLeaderEncoder.getVelocity();
    }

    // Return a tank drive command mode where each stick controls either side of the
    // motor. NOTE: robotDriveController.tankDrive squares the input
    public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
        return this.run(() -> {
            robotDriveController.tankDrive(left.getAsDouble(), right.getAsDouble());
        });
    }

    // Return an arcade drive command mode where one stick controls velocity and one
    // controls rotation NOTE: robotDriveController.arcadeDrive squares the input
    public Command arcadeDrive(DoubleSupplier velocity, DoubleSupplier rotation) {
        return this.run(() -> {
            robotDriveController.arcadeDrive(velocity.getAsDouble(), rotation.getAsDouble());
        });
    }

}
