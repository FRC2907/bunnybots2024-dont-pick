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
    double extensionSetPoint;
    double angleSetPoint;


    public Arm(){
        arm = new CANSparkMax(Ports.arm.ARM, MotorType.kBrushless);
        arm.getPIDController().setP(Control.arm.kAngleP);
        arm.getPIDController().setD(Control.arm.kAngleD);
        arm.getPIDController().setFF(Control.arm.kAngleFF);
        arm.getEncoder().setPositionConversionFactor(Control.arm.ENCODER_POS_UNIT_PER_DEGREE);
        arm.getEncoder().setVelocityConversionFactor(Control.arm.ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND);

        armExtend = new CANSparkMax(Ports.arm.ARM_EXTENSION, MotorType.kBrushless);
        armExtend.getPIDController().setP(Control.arm.kExtendP);
        armExtend.getPIDController().setD(Control.arm.kExtendD);
        armExtend.getPIDController().setFF(Control.arm.kExtendFF);
        armExtend.getEncoder().setPositionConversionFactor(Control.arm.ENCODER_POS_UNIT_PER_INCH);
        armExtend.getEncoder().setVelocityConversionFactor(Control.arm.ENCODER_VEL_UNIT_PER_INCH_PER_SECOND);
        
        extensionSetPoint = 0;

        angleSetPoint = 0;
    }

    private static Arm instance;

    public static Arm getInstance(){
        if(instance == null){
            instance = new Arm();
        }
        return instance;
    }

    public double getSetPointAngle(Translation2d _setPoint){
        double xDifference = _setPoint.getX() - Control.arm.ARM_POSITION.getX();
        double yDifference = _setPoint.getY() - Control.arm.ARM_POSITION.getY();
        double hypotenuse = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));

        return Math.asin(yDifference/hypotenuse);
    }

    public void setSetPoint(Translation2d _setPoint){
        double x = Util.clamp(Control.arm.ARM_POSITION.getX(), _setPoint.getX(), Control.arm.MAX_ARM_EXTENSION_SIDE);
        double y = Util.clamp(Control.arm.ARM_POSITION.getY(), _setPoint.getY(), Control.arm.MAX_ARM_EXTENSION_TOP);
        double hypotenuse = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double angle = getSetPointAngle(new Translation2d(x, y));

        angle = Util.clamp(Control.arm.MIN_ANGLE, angle, Control.arm.MAX_ANGLE);
        hypotenuse = Util.clamp(Control.arm.MIN_ARM_EXTENSION, hypotenuse, Control.arm.MAX_ARM_EXTENSION);

        angleSetPoint = angle;
        extensionSetPoint = hypotenuse;
    }

    public void setSetPoint(double angle){
        double x = Control.arm.MAX_ARM_EXTENSION_SIDE;
        double hypotenuse = x/Math.cos(angle);
        angle = Util.clamp(Control.arm.MIN_ANGLE, angle, Control.arm.MAX_ANGLE);
        hypotenuse = Util.clamp(Control.arm.MIN_ARM_EXTENSION, hypotenuse, Control.arm.MAX_ARM_EXTENSION);
        
        angleSetPoint = angle;
        extensionSetPoint = hypotenuse;
    }

    public void setSetPoint(double angle, double length) {
        length = Util.clamp(Control.arm.MIN_ARM_EXTENSION, length, Control.arm.MAX_ARM_EXTENSION);
        angle = Util.clamp(Control.arm.MIN_ANGLE, angle, Control.arm.MAX_ANGLE);

        angleSetPoint = angle;
        extensionSetPoint = length;
    }

    public void start() {
        setSetPoint(Control.arm.kStartAngle, Control.arm.kStartExtension);
    }

    public void neutral() {
        setSetPoint(Control.arm.kNeutralAngle, Control.arm.kNeutralExtension);
    }

    public void pickup(){
        setSetPoint(Control.arm.kPickup);
    }

    public void drop(){
        setSetPoint(Control.arm.kDropZone);
    }

    public void manualMove(double joystick){
        setSetPoint(joystick*(angleSetPoint+3));
    }


    @Override
    public void onLoop(){
        arm.getPIDController().setReference(angleSetPoint, ControlType.kPosition);
        armExtend.getPIDController().setReference(extensionSetPoint, ControlType.kPosition);
        receiveOptions();
        submitTelemetry();
    }
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
