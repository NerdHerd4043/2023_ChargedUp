// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.subsystems.CANdleSystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Slide;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.auto.BalanceOnPlatform;
import frc.robot.commands.autoCommands.PidBalance;
import frc.robot.commands.autoCommands.TimeDrive;
import frc.robot.commands.lightControl.PurpleLights;
import frc.robot.commands.lightControl.YellowLights;
import frc.robot.commands.slideCommands.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  // private final Arm arm = new Arm();

  private static CommandXboxController driveStick = new CommandXboxController(0);
  //private static XboxController driveStick = new XboxController(0);

  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  private PIDController pidController = new PIDController(AutoConstants.PID.kP, AutoConstants.PID.kI, AutoConstants.PID.kD);


  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  private final TimeDrive leaveCommunity = new TimeDrive(drivebase, -0.6, 3.5);
  private final PidBalance pidBalance = new PidBalance(
    drivebase, pidController, gyro,
    () -> Math.abs(limelightTable.getEntry("botpose").getDoubleArray(new Double[0])[0]));

  private final BalanceOnPlatform balanceOnPlatform = new BalanceOnPlatform(drivebase, slide, pidController, gyro);

  SendableChooser<Command> commandChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();

    commandChooser.addOption("Balance with PID", pidBalance);
    commandChooser.addOption("Leave the Community", leaveCommunity);
    commandChooser.addOption("Leave Community and Balance", balanceOnPlatform);

    SmartDashboard.putData(commandChooser);

    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> driveStick.getLeftY(),
            () -> driveStick.getRightX()));
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
    // new JoystickButton(driveStick, Button.kY.value).toggleOnTrue(new OpenSlide(Slide));
    driveStick.a().onTrue(new OpenSlide(slide));
    driveStick.y().onTrue(new CloseSlide(slide));
    driveStick.b().onTrue(new InstantCommand(drivebase::flipFront, drivebase));
    driveStick.start().onTrue(new InstantCommand(drivebase::setCoastMode, drivebase));
    driveStick.back().onTrue(new InstantCommand(drivebase::setBreakMode, drivebase));
    // driveStick.rightBumper().onTrue(new InstantCommand(arm::nextPose, arm));
    // driveStick.leftBumper().onTrue(new InstantCommand(arm::previousPose, arm));
    driveStick.povLeft().onTrue(new InstantCommand(candle::Purple, candle));
    driveStick.povRight().onTrue(new InstantCommand(candle::Yellow, candle));
  }
  
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return commandChooser.getSelected();
  }

  public Command getCandleOffCommand() {
    return new InstantCommand(candle::TurnOff);
  }

  public Command getCoastCommand(){
    return new InstantCommand(drivebase::setCoastMode);
  }

  public Command getBreakCommand(){
    return new InstantCommand(drivebase::setBreakMode);
  }
}
