package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class Stop extends CommandBase{
    private boolean isTurretOff;
    Turret turret;

    public Stop(Turret turret){
        addRequirements(turret);
        this.turret = turret;
    }

    public void exectue(){
        turret.hardStop();
    }

    public boolean TurretOff(){
        return isTurretOff;
    }
}
