// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.TankDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
  private final TankDriveSubsystem m_exampleSubsystem = new TankDriveSubsystem();

  private final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
  public double joyConLeft = 0;
  public double joyConRight = 0;
  // joy Con and PS5 methods copied from previous years tank drive

  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  
  private void configureBindings() {
    m_exampleSubsystem.setDefaultCommand(new RunCommand(() -> {
      this.joyConLeft = m_driverController.getLeftY(); // Left Y-axis for PS5 controller
      this.joyConRight = m_driverController.getRightX(); // Right Y-axis for PS5 controller
      m_exampleSubsystem.altDrive(joyConLeft, joyConRight);
    }, m_exampleSubsystem));
  }

  //this configure bindings was directly copied and pasted and altered at the parameters.


}
