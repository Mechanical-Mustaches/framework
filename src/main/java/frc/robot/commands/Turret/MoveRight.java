package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveRight extends CommandBase{
    private boolean isTurretRight;
    Turret turret;

    public MoveRight(Turret turret){
        addRequirements(turret);
        this.turret = turret;
    }

    public void execute(){
        turret.moveRight();
    }

    public boolean turretRight(){
        return isTurretRight;
    }

    public boolean isFinished(){
        return turret.isMovingRight();
    }
}
