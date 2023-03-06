// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class CANdleSystem extends SubsystemBase {
  ShuffleboardTab sd = Shuffleboard.getTab("SmartDashboard");

  GenericEntry cubeButton;
  GenericEntry coneButton;

  CANdle candle = new CANdle(CANdleConstants.CANdleID);

  /** Creates a new CANdle. */
  public CANdleSystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;

    ShuffleboardLayout ledContainer = sd
      .getLayout("LED Control", BuiltInLayouts.kList);

    cubeButton = ledContainer.add("Cube", false).getEntry();
    coneButton = ledContainer.add("Cone", false).getEntry();
  }

  public void turnPurple () {
    candle.setLEDs(255, 0, 255);
    SmartDashboard.putString("CANdle Light", "Cube");
  }

  public void turnYellow () {
    candle.setLEDs(255, 255, 0);
    SmartDashboard.putString("CANdle Light", "Cone");
  }

  public void turnOff() {
    candle.setLEDs(0,0,0);
    SmartDashboard.putString("CANdle Light", "CANdle Off");
  }
  
  public GenericEntry getCubeButtonEntry() {
    return cubeButton;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
