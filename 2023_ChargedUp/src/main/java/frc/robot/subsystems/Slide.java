// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlideConstants;

public class Slide extends SubsystemBase {
  private CANSparkMax slideMotor = new CANSparkMax(SlideConstants.slideMotorID, MotorType.kBrushless);

  // private boolean opened = false;
  private boolean closed = true;

  /** Creates a new Slide. */
  public Slide() {
    slideMotor.restoreFactoryDefaults(true);
    slideMotor.setSmartCurrentLimit(SlideConstants.currentLimit);
    slideMotor.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putBoolean("Door is Closed", closed);
  }

  public void open() {
    // if(!opened) {
      slideMotor.set(-SlideConstants.speed);
      // opened = true;
    // }
  }

  public void close() {
    // if(opened) {
      slideMotor.set(SlideConstants.speed);
      // opened = false;
    // }
  }

  public void driveSlideMotor(double speed) {
    slideMotor.set(speed);
  }

  public CommandBase driveSlideMotor(DoubleSupplier speed) {
    return this.run(() -> this.driveSlideMotor(speed.getAsDouble()));
  }

  public void stop() {
     slideMotor.stopMotor();
  }

  public void closeDoor() {
    closed = true;
  }

  public void openDoor() {
    closed = false;
  }

  public boolean isClosed() {
    return closed;
  }
  
  // public boolean isOpened() {
  //   return opened;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
