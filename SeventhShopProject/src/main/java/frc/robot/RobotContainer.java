// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;



public class RobotContainer {
  
CommandGenericHID keyboard = new CommandGenericHID(0);

// These button numbers are arbitrary; you can change them
Trigger keyQ = keyboard.button(5); // pretend "L1"
Trigger keyE = keyboard.button(6); // pretend "R1"

  private IntakeSubsystem intake = new IntakeSubsystem();

  private final CommandPS5Controller m_driverController = new CommandPS5Controller(0);

  public boolean l1;
  public boolean r1;
  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  
  private void configureBindings() {

    intake.setDefaultCommand(
      new RunCommand(() -> {
        boolean l1 =
            m_driverController.L1().getAsBoolean()
            || (RobotBase.isSimulation() && keyQ.getAsBoolean());
    
        boolean r1 =
            m_driverController.R1().getAsBoolean()
            || (RobotBase.isSimulation() && keyE.getAsBoolean());
    
            intake.moveArm(l1, r1);
      }, intake)
    );
    

  }

  //this configure bindings was directly copied and pasted and altered at the parameters.


}
