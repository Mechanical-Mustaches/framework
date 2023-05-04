package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


// Code
public class Elevator extends SubsystemBase {
    // Instance Variable
    private CANSparkMax elvtrMotor1 = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax elvtrMotor2 = new CANSparkMax(99, MotorType.kBrushless);
    private double motorSpeed = 0.5;

    // Constructor
    public Elevator() {
    
    }

    // Movement
    public void moveMotor() {
        elvtrMotor1.set(motorSpeed);
        elvtrMotor2.set(motorSpeed);
    }

    public void stop() {
        elvtrMotor1.set(0);
        elvtrMotor2.set(0);
    }

}
