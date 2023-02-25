// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cowlib.DualProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PID;
import static frc.robot.Constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

public class Arm extends DualProfiledPIDSubsystem {
  ShuffleboardTab shuffTab = Shuffleboard.getTab("Arm");

  private CANSparkMax lowerArmMotor = new CANSparkMax(ArmConstants.lowerArmMotorID, MotorType.kBrushless);
  private CANSparkMax upperArmMotor = new CANSparkMax(ArmConstants.upperArmMotorID, MotorType.kBrushless);

  private WPI_CANCoder lowerArmEncoder = new WPI_CANCoder(ArmConstants.lowerArmEncoderID);
  private WPI_CANCoder upperArmEncoder = new WPI_CANCoder(ArmConstants.upperArmEncoderID);

  private NetworkTableEntry lowerArmEncoderEntry;
  private NetworkTableEntry upperArmEncoderEntry;

  private double pose = 0;

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

    lowerArmMotor.restoreFactoryDefaults();
    upperArmMotor.restoreFactoryDefaults();

    lowerArmMotor.setIdleMode(IdleMode.kBrake);
    upperArmMotor.setIdleMode(IdleMode.kBrake);

    // PID Controllers in the Arm Tab
    ShuffleboardLayout pidContainer = shuffTab
      .getLayout("PID Controllers", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(0, 0);

    pidContainer.add("Lower PID Controller", m_controllerA);
    pidContainer.add("Upper PID Controller", m_controllerB);

    // Encoders in the Arm Tab
    ShuffleboardLayout encoderContainer = shuffTab
      .getLayout("Encoders", BuiltInLayouts.kList)
      .withSize(1, 2)
      .withPosition(2, 0);

    encoderContainer.add("Lower Arm Encoder", lowerArmEncoder);
    encoderContainer.add("Upper Arm Encoder", upperArmEncoder);
  }

  public double clamp(double value, double min, double max){
    return Math.max(min, Math.min(max, value));
  }

  public void nextPose() {
    changePose(1);
  }

  public void previousPose() {
    changePose(-1);
  }

  public void changePose(int nextPose) {
    pose = clamp(pose + nextPose, 0, poses.length - 1);
    updateGoals();
  }

  public void adjustPosition(double input) {
    pose = clamp(pose + input, 0, poses.length - 1);
    updateGoals();
  }

  public CommandBase adjustCommand(DoubleSupplier input) {
    return this.run(() -> this.adjustPosition(input.getAsDouble() / 100));
  }

  public void setGoals() {
    
  }

  public void updateGoals() {
    setGoals(poses[(int)pose].lower(), poses[(int)pose].upper());
  }

  @Override
  public void useOutput(double outputLower, double outputUpper, State setpointLower, State setpointUpper) {
    // lowerArmMotor.setVoltage(outputLower);
    // upperArmMotor.setVoltage(outputUpper);
  }

  @Override
  public double getMeasurement(Controller controller) {
    // Return the process variable measurement here
    switch(controller){
      case A: return lowerArmEncoder.getAbsolutePosition();
      case B: return upperArmEncoder.getAbsolutePosition();

      default: return 0;
    }
  }
}
