package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveLeft extends CommandBase{
    private boolean isMovingWrong;
    Turret turret;

    public MoveLeft(Turret turret){
        addRequirements(turret);
        this.turret = turret;
    }
    
    public void exectue(){
        turret.moveLeft();
    }

    public boolean is_MovingLeft(){
        return isMovingWrong;
    }

    public boolean isFinished(){
        return turret.isMovingLeft();
    }
}
