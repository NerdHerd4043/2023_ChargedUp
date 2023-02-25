// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib.arm;

/** Add your docs here. */
public class ArmPose {
    private double lower;
    private double upper;

    public ArmPose(double lower, double upper) {
      this.lower = lower;
      this.upper = upper;
    }

    public double lower() { return lower; }
    public double upper() { return upper; }
}
