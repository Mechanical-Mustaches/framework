package frc.robot.commands.floorIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntake;

public class FloorIntakeFartCommand extends CommandBase {
    FloorIntake floorIntake;

    public FloorIntakeFartCommand(FloorIntake floorIntake){
        addRequirements(floorIntake);
        this.floorIntake = floorIntake;
    }

    public void execute(){
        floorIntake.fartCube();
    }

    public boolean isFinished(){
        return floorIntake.isMunchCube();
    }

} // <-- keep brace 
