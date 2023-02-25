// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
  private CANSparkMax shoulderMotor = new CANSparkMax(ArmConstants.shoulderMotorID, MotorType.kBrushless);
  private CANSparkMax elbowMotor = new CANSparkMax(ArmConstants.elbowMotorID, MotorType.kBrushless);


  /** Creates a new Arm. */
  public Arm() {
    
      shoulderMotor.restoreFactoryDefaults(true);
      shoulderMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
      shoulderMotor.setIdleMode(IdleMode.kBrake);
      elbowMotor.restoreFactoryDefaults(true);
      elbowMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
      elbowMotor.setIdleMode(IdleMode.kBrake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
