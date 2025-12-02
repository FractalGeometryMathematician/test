// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

//change auto to use setposition instead of velocity
//manual not necessary


public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX leverMotor = new TalonFX( /*insert numer */ 3);
  private CANdi limit = new CANdi(/*insert number */ 2);
  private boolean setYet = false;
  private double upperLim = /*decide val */ 50; //check movearm to change value, 50 is just exorbitantly large random number, but check signage here
  private double invert = 1; //to reverse direction, just change 1 to -1
  private boolean goingUp = false;
  DoublePublisher pos;

  
  public IntakeSubsystem() {
    leverMotor.setPosition(0.0);
  }

  public void autoSetIntake(boolean left, boolean right){
    if(left && !right ){  
        leverMotor.set(-0.1*invert);
      
    }else if(!left && right){
        if(leverMotor.getPosition().getValueAsDouble() < upperLim){// check sign on upperLim
          leverMotor.set(0.1*invert);
        }
      
    }else{
      leverMotor.set(0.0);
    }

  }

  public void moveArm(boolean left, boolean right){
    if(!setYet){
      manual(left, right);
    }

    if(limit.getS1Closed().refresh().getValue()){ //check is bool is true or false when pressed
      setYet = true;
      leverMotor.setPosition(0.0); 
      leverMotor.set(0.0);
      upperLim = 2;                                       //check signage and number
    }

    if(leverMotor.getPosition().getValueAsDouble() > upperLim){//check signage
      leverMotor.set(0.0);
    }

    if(setYet){
      auto(left, right);
    }

  }


}
