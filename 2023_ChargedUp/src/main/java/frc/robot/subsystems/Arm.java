// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cowlib.DualProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PID;

public class Arm extends DualProfiledPIDSubsystem {

  private CANSparkMax lowerArmMotor = new CANSparkMax(ArmConstants.lowerArmMotorID, MotorType.kBrushless);
  private CANSparkMax upperArmMotor = new CANSparkMax(ArmConstants.upperArmMotorID, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
    super(
        //PID Controller A for lower arm
        new ProfiledPIDController(
          PID.Upper.kP,
          PID.Upper.kI,
          PID.Upper.kD,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(0, 0)),
        
        //PID Controller B for upper arm
        new ProfiledPIDController(
          PID.Lower.kP,
          PID.Lower.kI,
          PID.Lower.kD,
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
