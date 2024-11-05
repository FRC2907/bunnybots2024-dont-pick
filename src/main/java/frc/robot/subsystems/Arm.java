package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Ports;

public class Arm implements ISubsystem{
    CANSparkMax arm, armExtend;
    Translation2d setPoint;
    Translation2d armPosition;


    public Arm(){
        arm = new CANSparkMax(Ports.arm.ARM, MotorType.kBrushless);
        armExtend = new CANSparkMax(Ports.arm.ARM_EXTENSION, MotorType.kBrushless);
    }

    public void getSetPointAngle(Translation2d setpoint){
        double xDifference = setpoint.getX() - armPosition.getX();
        double yDifference = setpoint.getY() - armPosition.getY();
    }
}
