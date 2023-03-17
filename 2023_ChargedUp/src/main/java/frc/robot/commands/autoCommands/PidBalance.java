// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PidBalance extends PIDCommand {

  private final Drivebase drivebase;
  private final AHRS gyro;
  private double timerStart = 0;
  private double timerEnd = 1.5;
  private boolean firstCheck = true;

  /** Creates a new PidAuto. */
  public PidBalance(Drivebase drivebase, PIDController pidController, AHRS gyro, DoubleSupplier xPose) {
    super(
        // The controller that the command will use
        pidController,
        // This should return the measurement
        xPose,
        // This should return the setpoint (can also be a constant)
        () -> AutoConstants.chargeStationCenterPose,
        // This uses the output
        output -> {
          // Use the output here
          if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            if (gyro.getRoll() >= 6.5 && output > 0) { // this keeps the robot driving backwards when the aprltag comes into veiw
              drivebase.arcadeDrive(0.3, 0); // When the apriltag comes into view, the output defaults to a high positive number
            } 
            else { // else -> drive like normal with a max speed of 0.5
              drivebase.arcadeDrive(-Math.min(output, 0.35), 0); //.3
            }
          } 
          else {
            if (gyro.getRoll() >= 6.5) { // if the tag isn't seen and the charge station is tilted towards it, drive backwards                                  
              drivebase.arcadeDrive(0.4, 0);
            }
            else if (gyro.getRoll() <= -6.5) {  // if the tag isn't seen and the charge station is tilted away from it, drive forwards
              drivebase.arcadeDrive(-0.4, 0);
            }
            else{  // tag isn't visible on other occasions -> stop robot
              drivebase.stop();
            }
          }
          SmartDashboard.putNumber("PID Output", output);
        },
        drivebase);

    this.drivebase = drivebase;
    this.gyro = gyro;
    // this.timerStart = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (gyro.getRoll() >= -3 && gyro.getRoll() <= -1.5 &&
        Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
        .getDoubleArray(new Double[0])[0]) >= 4.1) {
      if (firstCheck) {
        timerStart = Timer.getFPGATimestamp();
        firstCheck = false;
      }
      return (Timer.getFPGATimestamp() - timerStart) > timerEnd;
    } else {
      firstCheck = true;
      return false;
    }
  }
}