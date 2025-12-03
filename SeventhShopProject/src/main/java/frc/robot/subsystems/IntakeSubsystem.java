// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

//current limits
//mutual vs braking mode
//torquecurrentfoc instead of voltage
//lower pos
//upperPos
//axis 0




public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX leverMotor = new TalonFX( /*insert numer */ 3, "can");
  private CANdi limit = new CANdi(/*insert number */ 2);
  private boolean autoOn = false;
  private final double upperLim = 3.5; //check movearm to change value, 50 is just exorbitantly large random number, but check signage here
  private double magnVel = 1; //to reverse direction, just change 1 to -1
  DoublePublisher pos;
  TalonFXConfiguration PID = new TalonFXConfiguration();
  NetworkTableInstance inst;
  NetworkTable table;
  private final double downPos = -1; //lower limit, in case angle of lever is lower. will be stopped by the limit anyway
  private boolean down = false; //current direction of arm
  
  

  
  public IntakeSubsystem() {
    leverMotor.setPosition(downPos);
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("data");
    pos = table.getDoubleTopic("pos").publish();
    config();
    
  }

  private void config(){
    PID.Slot0.kP = 0.01;                                                //fix these guys somehow
    PID.Slot0.kI = 0.0;                                                // config motors
    PID.Slot0.kV = 0.0;
    PID.Slot0.kD = 0.01;
    PID.Slot0.kG = 0.02;
    leverMotor.getConfigurator().apply(PID);
  }

  public void manual(boolean left, boolean right){      // manual mode
    if(left && right){                              //if both pressed, freeze motor at position, go to auto
      leverMotor.set(0*magnVel);
      leverMotor.setControl(new PositionVoltage(leverMotor.getPosition().getValueAsDouble()));
      autoOn = true;
      down = false;
      return;
    }

    if(left && !right ){                            //left pressed, go down
      leverMotor.set(-1*magnVel);
      down = true;
    }else if((!left && right) && (leverMotor.getPosition().getValueAsDouble() < upperLim)){ //right pressed, go up (unless too high already)
      leverMotor.set(1*magnVel);
      down = false;
    }else{                                          //none pressed, freeze
      leverMotor.set(0.0*magnVel);
      leverMotor.setControl(new PositionVoltage(leverMotor.getPosition().getValueAsDouble()));
      down = false;
    }
  }

  public void autoSetIntake(boolean left, boolean right){   //auto mode

    if(left && right){                                      //both pressed, freeze motor, go to auto mode
      leverMotor.set(0*magnVel);
      leverMotor.setControl(new PositionVoltage(leverMotor.getPosition().getValueAsDouble()));
      autoOn = false;
      down = false;
      return;
    }

    if(left && !right ){                                    //left pressed, go to downPos
        leverMotor.setControl(new PositionVoltage(downPos));
        down = true;
      
    }else if(!left && right){
        leverMotor.setControl(new PositionVoltage(upperLim));   //right pressed, go to up pos
        down = false;
    }

  }

  public void moveArm(boolean left, boolean right){

    if(limit.getS1Closed().refresh().getValue() && down ){ //check is bool is true or false when pressed. will go up if lim pressed, but not down
      leverMotor.set(0.0);
      leverMotor.setPosition(0.0); 
      leverMotor.setControl(new PositionVoltage(downPos));
      down = false;
    }


    pos.set(leverMotor.getPosition().getValueAsDouble()); //publish position of lever to network table

    if(!autoOn){                                          //go to respective method for movement
      manual(left, right);
    }

    if(autoOn){
      autoSetIntake(left, right);
    }

  }


}
