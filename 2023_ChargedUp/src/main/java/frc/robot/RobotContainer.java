// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Foot;
import frc.robot.subsystems.Slide;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.auto.BalanceOnPlatform;
import frc.robot.commands.auto.LeaveCommunity;
import frc.robot.commands.autoCommands.PidBalance;
import frc.robot.commands.autoCommands.TimeDrive;
import frc.robot.commands.slideCommands.*;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivebase drivebase = new Drivebase();
  private final Slide slide = new Slide();
  private final CANdleSystem candle = new CANdleSystem();
  private final Foot foot = new Foot();
  // private final Arm arm = new Arm();

  private static CommandXboxController driveStick = new CommandXboxController(0);
  private static CommandXboxController driveStick2 = new CommandXboxController(1);

  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  private PIDController pidController = new PIDController(AutoConstants.PID.kP, AutoConstants.PID.kI, AutoConstants.PID.kD);
  private MedianFilter filter = new MedianFilter(AutoConstants.medianFilter);

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  
  private final DoubleSupplier filteredXPose = 
    () -> filter.calculate(
      Math.abs(limelightTable.getEntry("botpose").getDoubleArray(new Double[0])[0]));

  // private final TimeDrive leaveCommunityTime = new TimeDrive(drivebase, -0.4, 2.6);
  // private final TimeDrive overChargeStation = new TimeDrive(drivebase, -0.3, 4);
  private final PidBalance pidBalance = new PidBalance(
    drivebase, pidController, gyro, filteredXPose);

  private final BalanceOnPlatform balanceOnPlatform = new BalanceOnPlatform(drivebase, slide, foot, pidController, gyro, filteredXPose);
  private final LeaveCommunity leaveCommunity = new LeaveCommunity(drivebase, slide);
  // private final SequentialCommandGroup balanceOnPlatform = new SequentialCommandGroup(
  //   new OpenSlide(slide),
  //   new WaitCommand(0.5),
  //   new CloseSlide(slide),
  //   new TimeDrive(drivebase, -0.3, 4),
  //   new WaitCommand(0.5),
  //   pidBalance,
  //   new InstantCommand(foot::down, foot)
  // );

  // private final SequentialCommandGroup leaveCommunity = new SequentialCommandGroup(
  //   new OpenSlide(slide),
  //   new WaitCommand(0.5),
  //   new CloseSlide(slide),
  //   new TimeDrive(drivebase, -0.4, 2.6)
  // );

  // private final SequentialCommandGroup scorePreload = new SequentialCommandGroup(
  //   new OpenSlide(slide),
  //   new WaitCommand(0.5),
  //   new CloseSlide(slide)
  // );


  SendableChooser<Command> commandChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();

    commandChooser.addOption("Balance with PID", pidBalance);
    commandChooser.addOption("Leave the Community", leaveCommunity);
    commandChooser.addOption("Leave Community and Balance", balanceOnPlatform);
    // commandChooser.addOption("Score Preload", scorePreload);

    SmartDashboard.putData(commandChooser);

    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> driveStick.getLeftY(),
            () -> driveStick.getRightX()));

    slide.setDefaultCommand(
      new SlideControl(
        slide, 
        () -> driveStick.getRightTriggerAxis()));
      }
    
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    driveStick.b().onTrue(new InstantCommand(slide::closeDoor, slide));
    driveStick.x().onTrue(new InstantCommand(foot::switchPosition, foot));
    driveStick.start().onTrue(new InstantCommand(drivebase::setCoastMode, drivebase));
    driveStick.back().onTrue(new InstantCommand(drivebase::setBreakMode, drivebase));
    driveStick.rightBumper().onTrue(new InstantCommand(candle::turnPurple, candle));
    driveStick.leftBumper().onTrue(new InstantCommand(candle::turnYellow, candle));

    driveStick2.rightBumper().onTrue(new InstantCommand(candle::turnPurple, candle));
    driveStick2.leftBumper().onTrue(new InstantCommand(candle::turnYellow, candle));
    // driveStick.rightBumper().onTrue(new InstantCommand(arm::nextPose, arm));
    // driveStick.leftBumper().onTrue(new InstantCommand(arm::previousPose, arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return commandChooser.getSelected();
  }

  public Command getCandleOffCommand() {
    return new InstantCommand(candle::turnOff, candle);
  }

  public Command getCoastCommand() {
    return new InstantCommand(drivebase::setCoastMode, drivebase);
  }

  public Command getBreakCommand() {
    return new InstantCommand(drivebase::setBreakMode, drivebase);
  }

  public Command getFootUpCommand() {
    return new InstantCommand(foot::up, foot);
  }
}
