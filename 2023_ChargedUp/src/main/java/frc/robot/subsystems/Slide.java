// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlideConstants;

public class Slide extends SubsystemBase {
  private CANSparkMax slideMotor = new CANSparkMax(SlideConstants.slideMotorID, MotorType.kBrushless);
  
  /** Creates a new Slide. */
  public Slide() {
    slideMotor.restoreFactoryDefaults(true);
    slideMotor.setSmartCurrentLimit(SlideConstants.currentLimit);
    slideMotor.setIdleMode(IdleMode.kBrake);
  }

  public void open () {
    slideMotor.set(SlideConstants.speed);
  }
  public void close () {
    slideMotor.set(-SlideConstants.speed);
  }
  public void stop () {
     slideMotor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
