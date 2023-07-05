package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Forward extends CommandBase {
    private boolean ShooterExtended;
    Shooter shooter;

    public Forward(Shooter shooter){
        addRequirements(shooter);
        this.shooter = shooter;
    }

    public void execute(){
        shooter.Forward();
    }

    public boolean isShooterExtended(){
        return ShooterExtended;
    }

}
