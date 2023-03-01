// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivebase;

public class Drive extends CommandBase {

  private final Drivebase drivebase;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotation;

  /** Creates a new Drive. */
  public Drive(Drivebase drivebase, DoubleSupplier forward, DoubleSupplier rotation) {
    this.drivebase = drivebase;
    this.forward = forward;
    this.rotation = rotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivebase.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble(), true, DriveConstants.speedLimit);
    drivebase.slewArcadeDrive(forward.getAsDouble(), rotation.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
