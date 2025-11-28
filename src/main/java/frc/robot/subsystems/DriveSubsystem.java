package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader = new SparkMax(DriveConstants.leftLeaderID, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(DriveConstants.leftFollowerID, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(DriveConstants.rightLeaderID, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(DriveConstants.rightFollowerID, MotorType.kBrushless);

    private final RelativeEncoder leftLeaderEncoder;
    private final RelativeEncoder rightLeaderEncoder;

    private SparkClosedLoopController leftLeaderController;
    private SparkClosedLoopController rightLeaderController;

    private DifferentialDrive robotDriveController;

    private SimpleMotorFeedforward velocityFeedforward = new SimpleMotorFeedforward(DriveConstants.kS,
            1 / DriveConstants.kV);

    private final AngularAccelerationUnit RPMPerSecond = RPM.per(Second);

    public DriveSubsystem() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        globalConfig
                .smartCurrentLimit((int) DriveConstants.currentLimit.in(Amps))
                .idleMode(IdleMode.kBrake).closedLoop
                .pid(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD).maxMotion
                .maxAcceleration(DriveConstants.maxAcceleration.in(RPMPerSecond))
                .maxVelocity(DriveConstants.maxVelocity.in(RPM))
                .allowedClosedLoopError(DriveConstants.allowedError);
        rightLeaderConfig
                .apply(globalConfig)
                .inverted(true);
        leftFollowerConfig
                .apply(globalConfig)
                .follow(leftLeader);

        rightFollowerConfig
                .apply(globalConfig)
                .follow(rightLeader);

        leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftLeaderController = leftLeader.getClosedLoopController();
        rightLeaderController = rightLeader.getClosedLoopController();

        leftLeaderEncoder = leftLeader.getEncoder();
        rightLeaderEncoder = rightLeader.getEncoder();

        robotDriveController = new DifferentialDrive(
                leftOutput -> setLeftVelocity(DriveConstants.maxMotorVelocity.times(leftOutput)),
                rightOutput -> setRightVelocity(DriveConstants.maxMotorVelocity.times(rightOutput)));
    }

    private void setLeftVelocity(AngularVelocity desiredVelocity) {
        Voltage arbFF = Volts.of(velocityFeedforward.calculate(desiredVelocity.in(RPM)));

        leftLeaderController.setReference(desiredVelocity.in(RPM), ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, arbFF.in(Volts));
    }

    private void setRightVelocity(AngularVelocity desiredVelocity) {
        Voltage arbFF = Volts.of(velocityFeedforward.calculate(desiredVelocity.in(RPM)));

        rightLeaderController.setReference(desiredVelocity.in(RPM), ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, arbFF.in(Volts));
    }

    public double getLeftVelocityRPM() {
        return leftLeaderEncoder.getVelocity();
    }

    public double getRightVelocityRPM() {
        return rightLeaderEncoder.getVelocity();
    }

    public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
        return this.run(() -> {
            robotDriveController.tankDrive(left.getAsDouble(), right.getAsDouble());
        });
    }

    public Command arcadeDrive(DoubleSupplier velocity, DoubleSupplier rotation) {
        return this.run(() -> {
            robotDriveController.arcadeDrive(velocity.getAsDouble(), rotation.getAsDouble());
        });
    }

}
