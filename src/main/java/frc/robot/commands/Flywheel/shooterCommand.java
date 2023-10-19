package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class shooterCommand extends CommandBase {
    Flywheel flywheel;
    private boolean is_FlywheelRolling;

    public shooterCommand(Flywheel flywheel){
        addRequirements(flywheel);
        this.flywheel = flywheel;
    }

    public void execute(){
        flywheel.Shoot();
    }

    public boolean isFlywheelRolling(){
        return is_FlywheelRolling;
    }
    
}
