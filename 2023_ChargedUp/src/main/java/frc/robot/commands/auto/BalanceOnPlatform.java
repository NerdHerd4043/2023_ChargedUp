// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.*;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceOnPlatform extends SequentialCommandGroup {
  /** Creates a new BalanceOnPlatform. */
  public BalanceOnPlatform(Drivebase drivebase, PIDController pidController, AHRS gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Drop piece
      new TimeDrive(drivebase, -0.6, 3.5),
      new PidBalance(
        drivebase, pidController, gyro,
        () -> Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new Double[0])[0]))
    );
  }
}