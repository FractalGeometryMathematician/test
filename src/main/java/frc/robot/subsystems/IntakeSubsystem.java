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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.security.Timestamp;


//mutual vs braking mode


public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX leverMotor = new TalonFX( /*insert numer */ 1, "can");
  private CANdi limit = new CANdi(/*insert number */ 3);
  private boolean autoOn = false;
  private final double upperLim = 3.5; //check movearm to change value, 50 is just exorbitantly large random number, but check signage here
  private double magnVel = 0.1; //to reverse direction, just change 1 to -1
  DoublePublisher pos;
  TalonFXConfiguration PID = new TalonFXConfiguration();
  NetworkTableInstance inst;
  NetworkTable table;
  private final double downPos = 1; //lower limit, in case angle of lever is lower. will be stopped by the limit anyway
  private boolean up = false; //current direction of arm
  CurrentLimitsConfigs currLim;
  private PositionTorqueCurrentFOC focThing;
  private VelocityTorqueCurrentFOC velFOCthing;
  private double haltUntil = 0;

  
  

  
  public IntakeSubsystem() {
    leverMotor.setPosition(upperLim);
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("data");
    pos = table.getDoubleTopic("pos").publish();
    config();
    
  }

  private void config(){
    PID.Slot0.kP = 3;                                                //fix these guys somehow
    PID.Slot0.kI = 0.1;                                                // config motors
    // PID.Slot0.kV = 1;
    PID.Slot0.kD = 0.01;
    PID.Slot0.kG = 0.02;

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

    leverMotor.getConfigurator().apply(PID);
    focThing = new PositionTorqueCurrentFOC(0).withSlot(0); //sets FOC object with PID values
    velFOCthing = new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0)).withSlot(1);
  
  }

  public void manual(boolean left, boolean right){      // manual mode
    // if(left && right){                              //if both pressed, freeze motor at position, go to auto
    //   leverMotor.setControl(focThing.withPosition(leverMotor.getPosition().getValueAsDouble()));
    //   autoOn = true;
    //   up = false;
    //   haltUntil = Timer.getFPGATimestamp() + 0.5;
    //   return;
    // }

    if(left && !right){                            //left pressed, go down
      // leverMotor.setControl(velFOCthing.withVelocity(RotationsPerSecond.of(-1*magnVel)));
      leverMotor.setControl(focThing.withPosition(upperLim));
      // leverMotor.set(-0.1);
      up = false;

    }else if((!left && right)){ //&& !(limit.getS1Closed().refresh().getValue())){ //right pressed, go up (unless too high already)
      // leverMotor.setControl(velFOCthing.withVelocity(RotationsPerSecond.of(1*magnVel)));
      leverMotor.setControl(focThing.withPosition(downPos));

      // leverMotor.set(0.1);
      up = true;

    }
    else{                                          //none pressed, freeze. alternatively, if going up but above upperLim, also stop
      leverMotor.setControl(focThing.withPosition(leverMotor.getPosition().getValueAsDouble()));
      up= false;
    }
  }

  public void autoSetIntake(boolean left, boolean right){   //auto mode

    if(left && right){                                      //both pressed, freeze motor, go to auto mode
      leverMotor.setControl(focThing.withPosition(leverMotor.getPosition().getValueAsDouble()));
      autoOn = false;
      up = false;
      haltUntil = Timer.getFPGATimestamp() + 0.5;
      return;
    }

    if(left && !right ){                                    //left pressed, go to downPos
        leverMotor.setControl(focThing.withPosition(downPos));
        up = false;
      
    }else if(!left && right){
      leverMotor.setControl(velFOCthing.withVelocity(RotationsPerSecond.of(1*magnVel)));   //right pressed, go to up pos
        up = true;
    }

  }

  public void moveArm(boolean left, boolean right){

    // if(limit.getS1Closed().refresh().getValue() && up ){ //check is bool is true or false when pressed. will go up if lim pressed, but not down
    //   leverMotor.setPosition(upperLim); 
    //   leverMotor.setControl(focThing.withPosition(upperLim));
    //   up = false;
    // }

    if(Timer.getFPGATimestamp() < haltUntil){
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
    pos.set(leverMotor.getPosition().getValueAsDouble()); //publish position of lever to network table

  }


}
