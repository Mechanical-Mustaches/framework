package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TankDrive extends SubsystemBase {
    // Instance variables
    private CANSparkMax frontRight = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax frontLeft = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax backRight = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax backLeft = new CANSparkMax(99, MotorType.kBrushless);

    private MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);
    private MotorControllerGroup right = new MotorControllerGroup(frontRight, backRight);

    private final DifferentialDrive drivetrain = new DifferentialDrive(left, right);

    private double maxSpd;
    private double maxTrn = 0.7;

    // Constructor
    public TankDrive(double speed) {
        maxSpd = speed;
    }

    // Movement
    public void drive() {
        drivetrain.arcadeDrive(maxSpd, maxTrn);
    }

}
