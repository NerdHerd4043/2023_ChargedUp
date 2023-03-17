// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveOverChargeStation extends CommandBase {
  Drivebase drivebase;
  AHRS gyro;
  double speed;
  double timerStart = 0;
  double timerEnd = 5;

  /** Creates a new DriveOverChargeStation. */
  public DriveOverChargeStation(Drivebase drivebase, AHRS gyro, double speed) {
    this.drivebase = drivebase;
    this.gyro = gyro;
    this.speed = speed;
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
    drivebase.arcadeDrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (gyro.getRoll() < -12) || ((Timer.getFPGATimestamp() - timerStart) > timerEnd);
  }
}
