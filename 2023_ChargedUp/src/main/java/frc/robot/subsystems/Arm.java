// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

// import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cowlib.DualProfiledPIDSubsystem;

public class Arm extends DualProfiledPIDSubsystem {
  private CANSparkMax lowerArmMotor = new CANSparkMax(lowerArmMotorID, MotorType.kBrushed);
  private CANSparkMax upperArmMotor = new CANSparkMax(upperArmMotorID, MotorType.kBrushed);

  private WPI_CANCoder lowerArmEncoder = new WPI_CANCoder(lowerArmMotorID);
  private WPI_CANCoder upperArmEncoder = new WPI_CANCoder(upperArmMotorID);

  private double currentPosition = 0;

  // Sim stuff start
  // private final DCMotor gearbox = DCMotor.getNEO(1);
  //
  // private final double lowerMotorReduction = 12;
  // private final double upperMotorReduction = 9;
  //
  // private final double lowerMass = Units.lbsToKilograms(5);
  // private final double upperMass = Units.lbsToKilograms(5);
  //
  // private final double lowerLength = Units.inchesToMeters(26.5);
  // private final double upperLength = Units.inchesToMeters(33);
  //
  // private final double lowerMinAngle = -120;
  // private final double lowerMaxAngle = 120;
  // private final double upperMinAngle = -175;
  // private final double upperMaxAngle = 175;

  // private final SingleJointedArmSim lowerArmSim =
  //     new SingleJointedArmSim(
  //             gearbox,
  //             lowerMotorReduction,
  //             SingleJointedArmSim.estimateMOI(lowerLength, lowerMass),
  //             lowerLength,
  //             Units.degreesToRadians(lowerMinAngle),
  //             Units.degreesToRadians(lowerMaxAngle),
  //             lowerMass,
  //             true);
  // private final SingleJointedArmSim upperArmSim =
  //     new SingleJointedArmSim(
  //             gearbox,
  //             upperMotorReduction,
  //             SingleJointedArmSim.estimateMOI(upperLength, upperMass),
  //             upperLength,
  //             Units.degreesToRadians(upperMinAngle),
  //             Units.degreesToRadians(upperMaxAngle),
  //             upperMass,
  //             true);


  // private final CANCoderSimCollection lowerEncoderSim = lowerArmEncoder.getSimCollection();
  // private final CANCoderSimCollection upperEncoderSim = upperArmEncoder.getSimCollection();

    // Sim stuff end

  private double lowerGoal;
  private double upperGoal;

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
  // https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html
  // to set these up I need a mechanism to characterize!!!! wooooO!!!!!!!
  // private SimpleMotorFeedforward lowerFeedForward = new SimpleMotorFeedforward(0, 0);
  // private SimpleMotorFeedforward upperFeedForward = new SimpleMotorFeedforward(0, 0);

  // private double lastLowerArmSpeed = 0;
  // private double lastUpperArmSpeed = 0;

  // private double lastTime = Timer.getFPGATimestamp();

  private Mechanism2d arm;
  private MechanismLigament2d lowerArm;
  private MechanismLigament2d upperArm;



  /** Creates a new Arm. */
  public Arm() {
    // Lower Arm = A
    // Upper Arm = B
    super(
        // The ProfiledPIDController used by the lower arm joint
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // The ProfiledPIDController used by the upper arm joint
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));
 
    updateGoals();

    arm = new Mechanism2d(3, 2);
    lowerArm = arm.getRoot("arm", 1, 0)
                  .append(new MechanismLigament2d("base", 0.5, 90, 5, new Color8Bit(Color.kDimGray)))
                  .append(new MechanismLigament2d("lowerArm", .85, 120, 5, new Color8Bit(Color.kCoral)));

    upperArm = lowerArm.append(new MechanismLigament2d("upperArm", 0.9, -175, 5, new Color8Bit(Color.kFuchsia)));

    SmartDashboard.putData("Army McArmFace", arm);

    lowerArmMotor.restoreFactoryDefaults(true);
    upperArmMotor.restoreFactoryDefaults(true);

  }

  @Override
  public void useOutput(double outputA, double outputB, TrapezoidProfile.State setpointA, TrapezoidProfile.State setpointB) {
    // Use the output (and optionally the setpoint) here
    lowerArmMotor.setVoltage(outputA);
    upperArmMotor.setVoltage(outputB); // need to add two arm feed forwards? spooky prospect
  }

  public void incrementPosition() {
    changePosition(1);
  }

  public void decrementPosition() {
    changePosition(-1);
  }

  public void changePosition(int amount) {
    currentPosition = clamp(
      Math.floor(currentPosition + amount),
      0,
      positions.length - 1
    );
    updateGoals(); 
  }

  public void adjustPosition(double amount) {
    currentPosition = clamp(
      currentPosition + amount,
      0,
      positions.length - .75
    );
    updateGoals();
  }

  // do the lerp
  private void updateGoals() {
    int lowerBound = Math.min((int) Math.floor(currentPosition), positions.length - 2);
    int upperBound = Math.min((int) Math.ceil(currentPosition), positions.length - 1);

    double fraction = currentPosition - lowerBound;

    // an optimization on bound markers, which admittedly dont happen super often
    // if (upperBound != lowerBound) {
        upperGoal = lerp(
            positions[lowerBound].upper(),
            positions[upperBound].upper(), 
            fraction);

        lowerGoal = lerp(
            positions[lowerBound].lower(),
            positions[upperBound].lower(),
            fraction);
    // } else {
    //     upperGoal = positions[upperBound].upper();
    //     lowerGoal = positions[upperBound].lower();
    // }

    setGoals(lowerGoal, upperGoal);
  }

  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private double lerp(double a, double b, double f) {
      return (a * (1.0 - f)) + (b * f);
  }

  public CommandBase adjustCommand(DoubleSupplier amount) {
    return this.run(() -> this.adjustPosition(amount.getAsDouble()));
  }

  @Override
  public void simulationPeriodic() {
    // upperArmSim.setInput(upperArmMotor.get() * RobotController.getBatteryVoltage());
    // lowerArmSim.setInput(lowerArmMotor.get() * RobotController.getBatteryVoltage());
    //
    // upperArmSim.update(0.20);
    // lowerArmSim.update(0.20);

    // upperArm.setAngle(Units.radiansToDegrees(upperArmSim.getAngleRads()));
    // lowerArm.setAngle(Units.radiansToDegrees(lowerArmSim.getAngleRads()));
    

    upperArm.setAngle(upperGoal);
    lowerArm.setAngle(lowerGoal);

    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(upperArmSim.getCurrentDrawAmps() + lowerArmSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    super.periodic();

    // upperArm.setAngle(upperGoal);
    // lowerArm.setAngle(lowerGoal);
    
    SmartDashboard.putNumber("Arm Position", currentPosition);
    SmartDashboard.putNumber("Lower Arm Goal", lowerGoal);
    SmartDashboard.putNumber("Upper Arm Goal", upperGoal);
  }

  @Override
  public double getMeasurement(Controller controller) {
    // Return the process variable measurement here
    switch(controller) {
      case A: return lowerArmEncoder.getAbsolutePosition();
      case B: return upperArmEncoder.getAbsolutePosition();
    }

    return 0;
  }
}
