// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ServoConstants;
//import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ManualServo;
import frc.robot.commands.ManualPivot;
//import frc.robot.commands.PositionServo;
//import frc.robot.commands.PositionPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StopperServo;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final PneumaticsSubsystem pneumaticSubsystem = new PneumaticsSubsystem();
  private final StopperServo intakeSubsystem = new StopperServo();


  // Controllers
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);


  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();


  }

  private void configureBindings() {
    // Square: 0° (stowed)
    //driverController.square().onTrue(
      //new PositionPivot(pivotSubsystem, PivotConstants.MIN_ANGLE)
    //);

    // Cross: 45° (mid position)
  //  driverController.cross().onTrue(
    //  new PositionPivot(pivotSubsystem, PivotConstants.MID_POSITION)
    //);

    // Circle: 90° (fully raised)
   // driverController.circle().onTrue(
     // new PositionPivot(pivotSubsystem, PivotConstants.MAX_ANGLE)
    //);
    //PNEUMATICS: Triangle
    driverController.triangle().onTrue(
      new InstantCommand(() -> pneumaticSubsystem.togglePneumatic(), pneumaticSubsystem)
    );

    //SERVO: Create button - move to open position (120 degrees)
    //driverController.create().onTrue(
      //new PositionServo(intakeSubsystem, ServoConstants.OPEN_POSITION) );

    //SERVO: R1 open (counterclockwise to 120°), L1 close (back to home 0°)
    driverController.R1().whileTrue(
      new ManualServo(intakeSubsystem, ServoConstants.INTAKE_SPEED)
    );

    driverController.L1().whileTrue(
      new ManualServo(intakeSubsystem, -ServoConstants.INTAKE_SPEED)
    );
}
  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
        // Drive base utilizes tank drive controls
    m_driveSubsystem.setDefaultCommand(
      m_driveSubsystem.tankDrive(
          () -> -m_driverController.getRightY(),
          () -> -m_driverController.getLeftY()));

  
    // Pivot Configs: R2 spins positive and L2 spins negative
      pivotSubsystem.setDefaultCommand(
      new ManualPivot(pivotSubsystem, () -> -driverController.getL2Axis() + driverController.getR2Axis()
      ));

}
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}