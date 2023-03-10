// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
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

  private SlewRateLimiter slewRate = new SlewRateLimiter(DriveConstants.slewRate);

  /** Creates a new Drivebase. */
  public Drivebase() {
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    
    frontLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    backLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    frontRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    backRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);

    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    backRightMotor.setIdleMode(IdleMode.kCoast);
    
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    frontLeftMotor.setInverted(false);
    backLeftMotor.setInverted(false);
    
    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    
    diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    SmartDashboard.putString("Motor Mode", "Coast");
  }

  public void arcadeDrive(double fwd, double rot, boolean sqrd, double speedLimit) {
    diffDrive.arcadeDrive(speedLimit * fwd, DriveConstants.turnLimit * rot, sqrd);
  }

  public void arcadeDrive(double fwd, double rot) {
    arcadeDrive(fwd, rot, false, 1);
  }

  public void slewArcadeDrive(double fwd, double rot, boolean sqrd) {
    diffDrive.arcadeDrive(slewRate.calculate(fwd), DriveConstants.turnLimit * rot, sqrd);
  }

  public void stop(){
    diffDrive.arcadeDrive(0, 0);
  }

  public void setCoastMode(){
    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    backRightMotor.setIdleMode(IdleMode.kCoast);
    SmartDashboard.putString("Motor Mode", "Coast");
  }

  public void setBreakMode(){
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putString("Motor Mode", "Break");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
