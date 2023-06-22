package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Conveyor extends SubsystemBase {
    private CANSparkMax cnvyrMotor1 = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax cnvyrMotor2 = new CANSparkMax(99, MotorType.kBrushless);

    // Constructor
    public Conveyor() {
        
    }

}
