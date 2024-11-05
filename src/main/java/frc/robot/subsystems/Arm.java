package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class Arm implements ISubsystem{
    CANSparkMax arm, armExtend;
    Translation2d setPoint;


    public Arm(){
        arm = new CANSparkMax(Ports.arm.ARM, MotorType.kBrushless);
        armExtend = new CANSparkMax(Ports.arm.ARM_EXTENSION, MotorType.kBrushless);
    }

    public void getSetPointAngle(Translation2d setpoint){
        double xDifference = setpoint.getX() - Control.arm.ARM_POSITION.getX();
        double yDifference = setpoint.getY() - Control.arm.ARM_POSITION.getY();
    }

    @Override
    public void onLoop(){
        receiveOptions();
        submitTelemetry();
    }
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
