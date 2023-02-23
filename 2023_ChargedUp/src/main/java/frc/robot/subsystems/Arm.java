// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cowlib.DualProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ArmConstants;;

public class Arm extends DualProfiledPIDSubsystem {
  /** Creates a new Arm. */
  public Arm() {
    super(
        //PID Controller A for lower arm
        new ProfiledPIDController(
          ArmConstants.ArmPIDConstants.kP1,
          ArmConstants.ArmPIDConstants.kI1,
          ArmConstants.ArmPIDConstants.kD1,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(0, 0)),
        
        //PID Controller B for upper arm
        new ProfiledPIDController(
          ArmConstants.ArmPIDConstants.kP2,
          ArmConstants.ArmPIDConstants.kI2,
          ArmConstants.ArmPIDConstants.kD2,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(0, 0)));
  }

  @Override
  public void useOutput(double outputA, double outputB, State setpointA, State setpointB) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement(Controller controller) {
    // Return the process variable measurement here
    return 0;
  }
}
