// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cowlib.arm.ArmPose;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int frontLeftMotorID = 25;
    public static final int backLeftMotorID = 24;
    public static final int frontRightMotorID = 10;
    public static final int backRightMotorID = 11;
    
    public static final int currentLimit = 40;
    
    public static final double turnLimit = 0.7;
  }

  public static class ArmConstants {
    public static final int lowerArmMotorID = 0;
    public static final int upperArmMotorID = 1;

    public static final int lowerArmEncoderID = 0;
    public static final int upperArmEncoderID = 1;

    public static final ArmPose[] poses = new ArmPose[] {
      new ArmPose(0, 0),
      new ArmPose(5, 5),
      new ArmPose(15, 20)
    };

    public static class PID {
      public static class Upper{
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }
      public static class Lower{
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }
    }

    public static class FeedForward {
      public static class Upper{
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
      }
      public static class Lower{
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
      }
    }
  }
  
  public static class SlideConstants {
    public static final int slideMotorID = 21;

    public static final int currentLimit = 3; 

    public static final double speed = 0.4;
  }

  public static final class AutoConstants {
    public static final double chargeStationCenterPose = 4.4; //X position of the center of the charge station
    
    public static final class PID {
      public static final double kP = 2;
      public static final double kI = 0;
      public static final double kD = 0.01;
    }
}

}
