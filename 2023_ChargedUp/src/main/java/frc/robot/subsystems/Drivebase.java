// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivebase extends SubsystemBase {
  private DifferentialDrive diffDrive;

  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);

  private boolean rslIsFront = true;

  /** Creates a new Drivebase. */
  public Drivebase() {
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    
    setCurrentLimit(DriveConstants.currentLimit);

    setCoastMode();
    
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    frontLeftMotor.setInverted(false);
    backLeftMotor.setInverted(false);
    
    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    
    diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    SmartDashboard.putBoolean("RSL Is Front", rslIsFront);
  }

  public void arcadeDrive(double fwd, double rot, boolean sqrd) {
    double direction = (rslIsFront) ? 1 : -1;

    diffDrive.arcadeDrive(fwd * direction, DriveConstants.turnLimit * rot, sqrd);
  }

  public void arcadeDrive(double fwd, double rot) {
    arcadeDrive(fwd, rot, false);
  }

  public void stop() {
    diffDrive.arcadeDrive(0, 0);
  }

  public void flipFront() {
    rslIsFront = !rslIsFront;
    SmartDashboard.putBoolean("RSL Is Front", rslIsFront);
  }

  public void setIdleMode(IdleMode mode) {
    frontLeftMotor.setIdleMode(mode);
    backLeftMotor.setIdleMode(mode);
    frontRightMotor.setIdleMode(mode);
    backRightMotor.setIdleMode(mode);
    SmartDashboard.putString("Motor Mode", mode.toString());
  }
  
  public void setCoastMode() {
    setIdleMode(IdleMode.kCoast);
  }

  public void setBreakMode() {
    setIdleMode(IdleMode.kBrake);
  }

  public void setCurrentLimit(int limit) {
    frontLeftMotor.setSmartCurrentLimit(limit);
    backLeftMotor.setSmartCurrentLimit(limit);
    frontRightMotor.setSmartCurrentLimit(limit);
    backRightMotor.setSmartCurrentLimit(limit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
