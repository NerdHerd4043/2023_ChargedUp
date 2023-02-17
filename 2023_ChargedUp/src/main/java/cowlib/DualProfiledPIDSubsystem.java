// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that uses two {@link ProfiledPIDController}s to control two outputs. The controllers are run
 * synchronously from the subsystem's periodic() method.
 *
 * <p>This class is provided with love by Zynh
 */
public abstract class DualProfiledPIDSubsystem extends SubsystemBase{
    /*
     * enum used throughout the DualProfiledPIDSubsystem class to select which controller to use
     */
    public enum Controller {
        A,
        B
    }

    protected final ProfiledPIDController m_controllerA;
    protected final ProfiledPIDController m_controllerB;

    protected boolean m_enabled;

    /**
     * Creates a new ProfiledPIDSubsystem.
     *
     * @param controllerA the {@link ProfiledPIDController} to act as controller A
     * @param controllerA the {@link ProfiledPIDController} to act as controller B
     * @param initialPosition the initial goal position of controller A
     * @param initialPosition the initial goal position of controller B
     */
    public DualProfiledPIDSubsystem(ProfiledPIDController controllerA, ProfiledPIDController controllerB, double initialPositionA, double initialPositionB) {
        this.m_controllerA = controllerA;
        this.m_controllerB = controllerB;
    }

    /**
     * Creates a new ProfiledPIDSubsystem. Initial goal position is zero.
     *
     * @param controllerA the ProfiledPIDController to act as controller A
     * @param controllerA the ProfiledPIDController to act as controller B
     */
    public DualProfiledPIDSubsystem(ProfiledPIDController m_controllerA, ProfiledPIDController m_controllerB) {
        this(m_controllerA, m_controllerB, 0, 0);
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(
                m_controllerA.calculate(getMeasurement(Controller.A)),
                m_controllerB.calculate(getMeasurement(Controller.B)),
                m_controllerA.getSetpoint(),
                m_controllerB.getSetpoint()
            );
        }
    }

    /**
     * @return both {@link ProfiledPIDController}s in an array [A, B]
     */
    public ProfiledPIDController[] getControllers() {
        return new ProfiledPIDController[] { m_controllerA, m_controllerB };
    }

    public ProfiledPIDController getController(Controller controller) {
        switch(controller) {
            case A: return m_controllerA;
            case B: return m_controllerB;

            default: return null;
        }
    }

    /**
     * Sets the goal states for one of the controllers of the subsystem. 
     * Goal velocity assumed to be zero.
     *
     * @param goalA The goal state for controller A's motion profile.
     * @param goalB The goal state for controller B's motion profile.
     */
    public void setGoals(State goalA, State goalB) {
        m_controllerA.setGoal(goalA);
        m_controllerB.setGoal(goalB);
    }

    /**
     * Sets the goal states for one of the controllers of the subsystem. 
     * Goal velocity assumed to be zero.
     *
     * @param goalA The goal position for controller A's motion profile.
     * @param goalB The goal position for controller B's motion profile.
     */
    public void setGoals(double goalA, double goalB) {
        setGoals(new State(goalA, 0), 
                 new State(goalB, 0));
    }

    /**
     * Sets the goal state for one of the controllers of the subsystem. 
     * Goal velocity assumed to be zero.
     *
     * @param goal The goal state for the subsystem's motion profile.
     * @param controller The controller to set the goal position of
     */
    public void setGoal(State goal, Controller controller) {
        switch(controller) {
            case A: m_controllerA.setGoal(goal); break;
            case B: m_controllerB.setGoal(goal); break;
        }
    }

    /**
     * Sets the goal state for one of the controllers of the subsystem. 
     * Goal velocity assumed to be zero.
     *
     * @param goal The goal position for the subsystem's motion profile.
     * @param controller The controller to set the goal position of
     */
    public void setGoal(double goal, Controller controller) {
        setGoal(new State(goal, 0), controller);
    }

    /**
     * Returns the measurement of the process variable used by the first ProfiledPIDController.
     *
     * @return the measurement of the process variable for controller A
     */
    protected abstract double getMeasurement(Controller controller);


    /**
     * Uses the output from the ProfiledPIDController.
     *
     * @param output the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController, for feedforward
     */
    protected abstract void useOutput(double outputA, double outputB, State setpointA, State setpointB);

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        m_enabled = true;

        m_controllerA.reset(getMeasurement(Controller.A));
        m_controllerB.reset(getMeasurement(Controller.B));
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        m_enabled = false;

        useOutput(0, 0, new State(), new State());
    }
    
    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return m_enabled;
    }
}
