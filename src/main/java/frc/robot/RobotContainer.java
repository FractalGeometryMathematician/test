// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MusicBoxSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class RobotContainer {

  // Subsystems
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  // private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  // private final MusicBoxSubsystem m_musicBoxSubsystem = new
  // MusicBoxSubsystem();

  // Driver controller
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    // Outtake uses velocity control with triggers
    m_outtakeSubsystem.setDefaultCommand(
        m_outtakeSubsystem.VelocityControl(
            () -> -m_driverController.getL2Axis(),
            () -> -m_driverController.getR2Axis()));

    // Drive base utilizes tank drive controls
    // m_driveSubsystem.setDefaultCommand(
    // m_driveSubsystem.tankDrive(
    // () -> -m_driverController.getLeftY(),
    // () -> -m_driverController.getRightY()));
  }

  private void configureBindings() {
    // D-Pad used to control step up and step down
    m_driverController.povUp().onTrue(m_outtakeSubsystem.stepUp());
    m_driverController.povDown().onTrue(m_outtakeSubsystem.stepDown());

    Trigger robotDisabled = new Trigger(RobotState::isDisabled);

    // robotDisabled.and(m_driverController.povUp())
    // .onTrue(m_musicBoxSubsystem.selectNextSong());
    // robotDisabled.and(m_driverController.povDown())
    // .onTrue(m_musicBoxSubsystem.selectPreviousSong());
    // robotDisabled.and(m_driverController.cross())
    // .onTrue(m_musicBoxSubsystem.togglePlayPause());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
