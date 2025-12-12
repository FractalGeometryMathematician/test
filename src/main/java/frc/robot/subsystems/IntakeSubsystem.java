// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.security.Timestamp;
import java.util.EnumSet;


//mutual vs braking mode


public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX leverMotor = new TalonFX( /*insert numer */ 1, "can");
  private CANdi limit = new CANdi(/*insert number */ 3);
  private boolean autoOn = true;
  private final double upperLim = 3.5; //check movearm to change value, 50 is just exorbitantly large random number, but check signage here
  private double magnVel = 0.05; //to reverse direction, just change 1 to -1
  DoublePublisher pos;
  TalonFXConfiguration PID = new TalonFXConfiguration();
  NetworkTableInstance inst;
  NetworkTable table;
  private final double downPos = -1.0; //lower limit, in case angle of lever is lower. will be stopped by the limit anyway
  private boolean up = false; //current direction of arm
  CurrentLimitsConfigs currLim;
  private PositionTorqueCurrentFOC focThing;
  private VelocityTorqueCurrentFOC velFOCthing;
  private double haltUntil = 0;
  private final double torqLim = 15;
  private MotionMagicTorqueCurrentFOC slo;
  private double holdPos;

  
  

  
  public IntakeSubsystem() {
    holdPos=leverMotor.getPosition().getValueAsDouble();
    leverMotor.setPosition(upperLim);
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("data");
    pos = table.getDoubleTopic("pos").publish();
    config();
    configNT();
    
  }
  public void configPID(double p, double i, double d, double ff) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values
        /*
         * Think of P as how much we want it to correct, as an example imagine you are parking a car
         */

        slot0Configs.kP = p;
        /*
         * Integral Control's job is to correct recurring errors over time by stacking past errors.
         * It sums up previous errors, so it looks at how many errors you have had over time.
         */
        slot0Configs.kI = i;

        /*
         * The Derivitive Controls job is to look at the Rate of Change (slope) of how fast the error is changing (def of derrivitive)
         * If the error is chaning too fast, the kD will slow it down so we do not overshoot
         */
        slot0Configs.kD = d;

        /*
        * Feedforward Control (kFF, or kV in Phoenix 6) predicts how much power we need based only on how fast we want to go,
        *      instead of waiting for an error to happen first.
        */
        slot0Configs.kG = ff;
        
        leverMotor.getConfigurator().apply(slot0Configs);
    }


  private void configNT(){
    NetworkTableInstance.getDefault().getTable("intakeDEBUG")
            .getEntry("PIDF")
            .setDoubleArray(
                new double[] {
                    5,
                    1,
                    1,
                    0.5
                }
            );
    NetworkTableInstance.getDefault().getTable("intakeDEBUG").addListener(
             "PIDF",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                double[] pidf = event.valueData.value.getDoubleArray();
                configPID(pidf[0], pidf[1], pidf[2], pidf[3]);
            }
        );
  }


  private void config(){
    PID.Slot0.kP = 3;                                                //fix these guys somehow
    PID.Slot0.kI = 0.1;                                                // config motors
    // PID.Slot0.kV = 1;
    PID.Slot0.kD = 0.01;
    // PID.Slot0.kG = 0.02;

    PID.Slot1.kP = 0.2;    // velocity foc
    PID.Slot1.kI = 0.0;
    PID.Slot1.kD = 0.0;
    PID.Slot1.kV = 1.0;   


    currLim = new CurrentLimitsConfigs()                             // current limits for safety
      .withStatorCurrentLimit(50.0)              
      .withStatorCurrentLimitEnable(true);
    PID.withCurrentLimits(currLim);

    PID.MotorOutput.NeutralMode = NeutralModeValue.Brake;          //neutral mode added, will brake automatically
    PID.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PID.MotionMagic.MotionMagicCruiseVelocity = 0.05;  // max speed (rotations/sec)
    PID.MotionMagic.MotionMagicAcceleration   = 1;  // how fast you ramp to that speed
    PID.MotionMagic.MotionMagicJerk           = 0.0;
    
    leverMotor.getConfigurator().apply(PID);
    focThing = new PositionTorqueCurrentFOC(0).withSlot(0); //sets FOC object with PID values
    velFOCthing = new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0)).withSlot(1);
    slo = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  
  }

  public void manual(boolean left, boolean right){      // manual mode
    
     if(left && !right){                            //left pressed, go down
      leverMotor.set(-1*magnVel);
      up = false;
      holdPos = leverMotor.getPosition().getValueAsDouble();
      
    }else if((!left && right)){ 
      leverMotor.set(magnVel);
      up = true;
      holdPos = leverMotor.getPosition().getValueAsDouble();
      
    }else{                                          //none pressed, freeze. alternatively, if going up but above upperLim, also stop
      leverMotor.set(0);
      leverMotor.setControl(focThing.withPosition(holdPos));
      up = false;

    }
  }

  public void autoSetIntake(boolean left, boolean right){   //auto mode

    /* 
    if(left && right){                                      //both pressed, freeze motor, go to auto mode
      leverMotor.setControl(focThing.withPosition(leverMotor.getPosition().getValueAsDouble()));
      autoOn = false;
      up = false;
      haltUntil = Timer.getFPGATimestamp() + 0.5;
      return;
    }
      */

    if(left && !right ){                                    //left pressed, go to downPos
        leverMotor.setControl(slo.withPosition(downPos));
        up = false;
        holdPos = downPos;
      
    }else if(!left && right){
      
        leverMotor.setControl(focThing.withPosition(upperLim));   //right pressed, go to up pos
        up = true;
        holdPos = upperLim;
      
    }

  }

  public void moveArm(boolean left, boolean right){

    // if(limit.getS1Closed().refresh().getValue() && up ){ //check is bool is true or false when pressed. will go up if lim pressed, but not down
    //   leverMotor.setPosition(upperLim); 
    //   // leverMotor.setControl(focThing.withPosition(upperLim));
    //   // up = false;
    // }

    if(Timer.getFPGATimestamp() < haltUntil){
      return;
    }

    if(left && right){                                      //both pressed, freeze motor, go to auto mode
      leverMotor.setControl(focThing.withPosition(leverMotor.getPosition().getValueAsDouble()));
      autoOn = !autoOn;
      up = false;
      haltUntil = Timer.getFPGATimestamp() + 0.5;
      holdPos = leverMotor.getPosition().getValueAsDouble();
      return;
    }


    if(!autoOn){                                          //go to respective method for movement
      manual(left, right);
    }

    if(autoOn){
      autoSetIntake(left, right);
    }

  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("lever motor position",leverMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("lever motor target",leverMotor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putBoolean("autoOn",autoOn);
    SmartDashboard.putBoolean("limitOn", limit.getS1Closed().refresh().getValue());

    pos.set(leverMotor.getPosition().getValueAsDouble()); //publish position of lever to network table

  }


}
