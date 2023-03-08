package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FootConstants;

public class Foot extends SubsystemBase {
    private Solenoid foot = new Solenoid(PneumaticsModuleType.CTREPCM, FootConstants.footSolenoidId);
    private boolean footUp = true;

    public void down() {
        foot.set(true);
    }

    public void up() {
        foot.set(false);
    }

    public void switchPosition() {
        footUp = !footUp;
        if(footUp){down();}
        else{up();}
    }
}
