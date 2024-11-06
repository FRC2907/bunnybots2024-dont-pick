package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class Arm implements ISubsystem{
    CANSparkMax arm, armExtend;
    Translation2d setPoint;


    public Arm(){
        arm = new CANSparkMax(Ports.arm.ARM, MotorType.kBrushless);
        armExtend = new CANSparkMax(Ports.arm.ARM_EXTENSION, MotorType.kBrushless);
    }

    public double getSetPointAngle(Translation2d _setPoint){
        double xDifference = _setPoint.getX() - Control.arm.ARM_POSITION.getX();
        double yDifference = _setPoint.getY() - Control.arm.ARM_POSITION.getY();
        double hypotenuse = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));

        return Math.asin(yDifference/hypotenuse);
    }

    public void setSetPoint(Translation2d _setPoint){
        double x = Util.clamp(0, _setPoint.getX(), Control.arm.MAX_ARM_EXTENSION_SIDE);
        double y = Util.clamp(0, _setPoint.getY(), Control.arm.MAX_ARM_EXTENSION_TOP);
        double hypotenuse = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double angle = getSetPointAngle(new Translation2d(x, y));

        arm.getPIDController().setReference(angle, ControlType.kPosition);
        armExtend.getPIDController().setReference(hypotenuse, ControlType.kPosition);
    }

    public void setSetPoint(double angle){
        double x = Control.arm.MAX_ARM_EXTENSION_SIDE;
        double hypotenuse = x/Math.cos(angle);
        
        arm.getPIDController().setReference(angle, ControlType.kPosition);
        armExtend.getPIDController().setReference(hypotenuse, ControlType.kPosition);
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
