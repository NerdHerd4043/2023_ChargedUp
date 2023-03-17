package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FootConstants;

public class Foot extends SubsystemBase {
    private Solenoid foot = new Solenoid(PneumaticsModuleType.CTREPCM, FootConstants.footSolenoidId);
    private boolean footUp = true;

    public Foot() {
        up();
    }

    public void down() {
        foot.set(true);
        footUp = false;
        SmartDashboard.putBoolean("Feet are Up", footUp);
    }

    public void up() {
        foot.set(false);
        footUp = true;
        SmartDashboard.putBoolean("Feet are Up", footUp);
    }

    public void switchPosition() {
        if(footUp){down();}
        else{up();}
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
