package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final SparkMax leftFront = new SparkMax(DriveConstants.left_front_motor_id, MotorType.kBrushless);
    private final SparkMax leftBack = new SparkMax(DriveConstants.left_back_motor_id, MotorType.kBrushless);
    private final SparkMax rightFront = new SparkMax(DriveConstants.right_front_motor_id, MotorType.kBrushless);
    private final SparkMax rightBack = new SparkMax(DriveConstants.right_back_motor_id, MotorType.kBrushless);

    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftBackConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightBackConfig = new SparkMaxConfig();

    public static enum Side {
        LEFT,
        RIGHT
    }

    public DriveSubsystem() {
        motorConfig.inverted(false);
        motorConfig.idleMode(IdleMode.kCoast);
        leftConfig.inverted(true);
        leftBackConfig.follow(leftFront);
        rightBackConfig.follow(rightFront);

        configureMotor(leftFront, motorConfig, leftConfig);
        configureMotor(leftBack, motorConfig, leftConfig, leftBackConfig);
        configureMotor(rightFront, motorConfig);
        configureMotor(rightBack, motorConfig, rightBackConfig);
    }

    private void configureMotor(SparkMax motor, SparkBaseConfig... configs) {
        for (SparkBaseConfig config : configs) {
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    private SparkMax getMotorObject(Side whichMotor) {
        if (whichMotor == Side.LEFT) {
            return leftFront;
        } else {
            return rightFront;
        }
    }

    public void setMotorSpeed(Dimensionless speed, Side whichMotor) {
        getMotorObject(whichMotor).set(speed.in(Value));
    }

    public Dimensionless getMotorSpeed(Side whichMotor) {
        return Value.of(getMotorObject(whichMotor).get());
    }

    public void setBothMotors(Dimensionless joyConLeft, Dimensionless joyConRight) {
        setMotorSpeed(joyConLeft, DriveSubsystem.Side.LEFT);
        setMotorSpeed(joyConRight, DriveSubsystem.Side.RIGHT);
    }
}
