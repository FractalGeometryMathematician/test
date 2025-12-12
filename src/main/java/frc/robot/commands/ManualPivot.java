// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import java.util.function.DoubleSupplier;

public class ManualPivot extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final DoubleSupplier speedSupplier;

   /**
   * Creates a new ManualPivot command
   *
   * @param pivotSubsystem Pivot subsystem
   * @param speedSupplier A supplier returning the desired speed (-1.0 to 1.0), in this case its R2 and L2
   *
   */
  public ManualPivot(PivotSubsystem pivotSubsystem, DoubleSupplier speedSupplier) {
    this.pivotSubsystem = pivotSubsystem;
    this.speedSupplier = speedSupplier;
    addRequirements(pivotSubsystem);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    // Allow this command to be interrupted by button commands
    return InterruptionBehavior.kCancelSelf;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Get the current speed value from R2 and L2
    double speedValue = speedSupplier.getAsDouble();
    // Apply configured speed scaling for safer manual control
    double scaledSpeed = speedValue * frc.robot.Constants.PivotConstants.PIVOT_MANUAL_SPEED;
    pivotSubsystem.setManualSpeed(scaledSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}