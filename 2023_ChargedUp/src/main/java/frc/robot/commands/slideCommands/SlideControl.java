// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.slideCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;

public class SlideControl extends CommandBase {

  private final Slide slide;
  private final DoubleSupplier speed;

  /** Creates a new SlideControl. */
  public SlideControl(Slide slide, DoubleSupplier speed) {
    this.slide = slide;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.slide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(slide.isClosed()) {
      slide.driveSlideMotor(0.2);

      if(speed.getAsDouble() > 0.05) {
        slide.openDoor();
      }
    }
    else {
      slide.driveSlideMotor(-speed.getAsDouble() * 0.2);
    }
    SmartDashboard.putBoolean("Door is Closed", slide.isClosed());
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
