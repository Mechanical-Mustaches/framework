package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Conveyor extends SubsystemBase {
    //Instance Variables
    private String name;
   // private CANSparkMax motor = new CANSparkMax(98, MotorType.kBrushless);
    private Double speed;
    RelativeEncoder encoder;

    //Constructor
    public Conveyor(String name, Double speed){
        this.name = name;
        this.speed = speed;
    }

    //Starting Command
    public void initialize(){
        System.out.println(name + "is booting up");
    }

    //Movement
    public void Forward(){
     //   motor.set(speed);
    }

    public void Reverse(){
     //   motor.set(-speed);
    }

    public void Stop(){
      //  motor.set(0);
    }

    

}
