// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix.led.CANdleConfiguration;
import cowlib.arm.ArmPosition;


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
    
    public static final int currentLimit = 30;
    public static final double slewRate = 2.2; //4 without slide, 2.2 with slide
    
    public static final double speedLimit = 1;
    public static final double turnLimit = 0.6;
  }

  public static class ArmConstants {
    public static final int lowerArmMotorID = 0;
    public static final int upperArmMotorID = 1;

    public static final int lowerArmEncoderID = 0;
    public static final int upperArmEncoderID = 1;

    public static final ArmPosition[] poses = new ArmPosition[] {
      new ArmPosition(0, 0),
      new ArmPosition(5, 5)
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
    public static final int slideMotorID = 30;

    public static final int currentLimit = 1; 

    public static final double speed = 0.4;
  }

  public static final class FootConstants {
    public static final int footSolenoidId = 3;
  }

  public static final class AutoConstants {
    public static final double chargeStationCenterPose = 4.4; //X position of the center of the charge station
    //At comp: 4.4, at shop: 4.7
    
    public static final int medianFilter = 5;

    public static final class PID {
      public static final double kP = 0.5;
      public static final double kI = 0;
      public static final double kD = 0.0098;
    }
  }
  
  public static final class CANdleConstants {
    public static final int CANdleID = 1;
  }
}
