// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.slideCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;

public class CloseSlide extends CommandBase {

  private final Slide slide;
  private double startTime;

  /** Creates a new OpenSlide. */
  public CloseSlide(Slide slide) {
    this.slide = slide;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.slide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    slide.close();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    slide.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > startTime + 0.5;
  }
}
