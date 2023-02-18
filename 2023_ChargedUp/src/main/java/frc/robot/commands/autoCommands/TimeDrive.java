// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class TimeDrive extends CommandBase {
  Drivebase drivebase;
  double speed;
  double timerStart;
  double timerEnd;

  /** Creates a new TimeDrive. */
  public TimeDrive(Drivebase drivebase, double speed, double waitTime) {
    this.drivebase = drivebase;
    this.speed = speed;
    timerStart = 0;
    timerEnd = waitTime;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.arcadeDrive(-speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - timerStart) > timerEnd;
  }
}