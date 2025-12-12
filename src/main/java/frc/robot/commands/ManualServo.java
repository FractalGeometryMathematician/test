// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StopperServo;

public class ManualServo extends Command {
  private final StopperServo intakeSubsystem;
  private final double speed;

  /**
   * Creates new ManualServo command
   * 
   * @param intakeSubsystem The stopper servo subsystem to control
   * @param speed The speed/direction to move the servo (-1.0 to 1.0)
   */
  public ManualServo(StopperServo intakeSubsystem, double speed) {
    this.intakeSubsystem = intakeSubsystem;
    this.speed = speed;
    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    intakeSubsystem.move(speed);
    // Continuously moves servo at the specified speed 
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}