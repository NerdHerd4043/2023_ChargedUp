// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cowlib.DualProfiledPIDSubsystem;

public class Arm extends DualProfiledPIDSubsystem {
  private CANSparkMax lowerArmMotor = new CANSparkMax(ArmConstants.lowerArmMotorID, MotorType.kBrushed);
  private CANSparkMax upperArmMotor = new CANSparkMax(ArmConstants.upperArmMotorID, MotorType.kBrushed);

  private CANCoder lowerArmEncoder = new CANCoder(ArmConstants.lowerArmMotorID);
  private CANCoder upperArmEncoder = new CANCoder(ArmConstants.upperArmMotorID);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
  // https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html
  // to set these up I need a mechanism to characterize!!!! wooooO!!!!!!!
  // private SimpleMotorFeedforward lowerFeedForward = new SimpleMotorFeedforward(0, 0);
  // private SimpleMotorFeedforward upperFeedForward = new SimpleMotorFeedforward(0, 0);

  // private double lastLowerArmSpeed = 0;
  // private double lastUpperArmSpeed = 0;

  // private double lastTime = Timer.getFPGATimestamp();


  /** Creates a new Arm. */
  public Arm() {
    // Lower Arm = A
    // Upper Arm = B
    super(
        // The ProfiledPIDController used by the lower arm joint
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // The ProfiledPIDController used by the upper arm joint
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));
  }

  @Override
  public void useOutput(double outputA, double outputB, TrapezoidProfile.State setpointA, TrapezoidProfile.State setpointB) {
    // Use the output (and optionally the setpoint) here
    lowerArmMotor.setVoltage(outputA);
    upperArmMotor.setVoltage(outputB); // need to add two arm feed forwards? spooky prospect
  }

  @Override
  public double getMeasurement(Controller controller) {
    // Return the process variable measurement here
    switch(controller) {
      case A: return lowerArmEncoder.getAbsolutePosition();
      case B: return upperArmEncoder.getAbsolutePosition();
    }

    return 0;
  }
}
