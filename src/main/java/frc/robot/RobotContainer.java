// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.MusicBoxSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // Subsystems
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  // private final MusicBoxSubsystem m_musicBoxSubsystem = new
  // MusicBoxSubsystem();

  // Intake Controller Booleans
  public boolean l1;
  public boolean r1;

  // Driver controller
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Drive base utilizes tank drive controls
    m_driveSubsystem.setDefaultCommand(
        m_driveSubsystem.arcadeDrive(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX()));

    m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> {
      this.l1 = m_driverController.L1().getAsBoolean(); // Left Y-axis for PS5 controller
      this.r1 = m_driverController.R1().getAsBoolean();
      m_intakeSubsystem.moveArm(l1, r1);

    }, m_intakeSubsystem));

    // D-Pad used to control step up and step down
    m_driverController.povUp().onTrue(m_outtakeSubsystem.stepUp());
    m_driverController.povDown().onTrue(m_outtakeSubsystem.stepDown());

    // Outtake uses velocity control with triggers
    m_outtakeSubsystem.setDefaultCommand(
        m_outtakeSubsystem.valueControl(
            () -> -m_driverController.getL2Axis(),
            () -> -m_driverController.getR2Axis()));

    // Only use music box when robot is disabled
    // Trigger robotDisabled = new Trigger(RobotState::isDisabled);
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
