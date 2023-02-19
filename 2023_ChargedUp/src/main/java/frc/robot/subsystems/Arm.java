// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

// import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cowlib.DualProfiledPIDSubsystem;

public class Arm extends DualProfiledPIDSubsystem {
  private CANSparkMax lowerArmMotor = new CANSparkMax(lowerArmMotorID, MotorType.kBrushed);
  private CANSparkMax upperArmMotor = new CANSparkMax(upperArmMotorID, MotorType.kBrushed);

  private CANCoder lowerArmEncoder = new CANCoder(lowerArmMotorID);
  private CANCoder upperArmEncoder = new CANCoder(upperArmMotorID);

  private double currentPosition = 0;

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

    lowerArmMotor.restoreFactoryDefaults(true);
    upperArmMotor.restoreFactoryDefaults(true);    
  }

  @Override
  public void useOutput(double outputA, double outputB, TrapezoidProfile.State setpointA, TrapezoidProfile.State setpointB) {
    // Use the output (and optionally the setpoint) here
    // lowerArmMotor.setVoltage(outputA);
    // upperArmMotor.setVoltage(outputB); // need to add two arm feed forwards? spooky prospect
  }

  public void incrementPosition() {
    changePosition(1);
  }

  public void decrementPosition() {
    changePosition(-1);
  }

  public void changePosition(int amount) {
    currentPosition = clamp(
      Math.floor(currentPosition + amount),
      0,
      positions.length - 1
    );
    updateGoals(); 
  }

  public void adjustPosition(double amount) {
    currentPosition = clamp(
      currentPosition + amount,
      0,
      positions.length - .75
    );
    updateGoals();
  }

  // do the lerp
  private void updateGoals() {
    int lowerBound = Math.min((int) Math.floor(currentPosition), positions.length - 2);
    int upperBound = Math.min((int) Math.ceil(currentPosition), positions.length - 1);

    double fraction = currentPosition % Math.floor(currentPosition);

    setGoals(
      lerp(
        positions[lowerBound].upper(),
        positions[upperBound].upper(), 
        fraction),
      lerp(
        positions[lowerBound].lower(),
        positions[upperBound].lower(),
        fraction) 
    );
  }

  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private double lerp(double a, double b, double f) {
      return (a * (1.0 - f)) + (b * f);
  }

  public CommandBase adjustCommand(DoubleSupplier amount) {
    return this.run(() -> this.adjustPosition(amount.getAsDouble()));
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
