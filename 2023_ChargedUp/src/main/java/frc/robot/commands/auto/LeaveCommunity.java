// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autoCommands.TimeDrive;
import frc.robot.commands.slideCommands.CloseSlide;
import frc.robot.commands.slideCommands.OpenSlide;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Slide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveCommunity extends SequentialCommandGroup {
  /** Creates a new leaveCommunity. */
  public LeaveCommunity(Drivebase drivebase, Slide slide) {
    // Add your commands in the addcommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OpenSlide(slide),
      new WaitCommand(0.5),
      new CloseSlide(slide),
      new TimeDrive(drivebase, -0.4, 2.6)
    );
  }
}
