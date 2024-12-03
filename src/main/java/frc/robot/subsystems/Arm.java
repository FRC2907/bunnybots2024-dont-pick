package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Util;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class Arm implements ISubsystem{
    private CANSparkMax arm, armExtend;
    private double extensionSetPoint, angleSetPoint;
    private SimpleMotorFeedforward extendFeedforward;
    private ArmFeedforward angleFeedforward;
    private PIDController anglePID, extendPID;
    private RelativeEncoder angleEncoder, extendEncoder;

    public Arm(){
        arm = new CANSparkMax(Ports.arm.ARM, MotorType.kBrushless);
        angleEncoder = arm.getEncoder();
        angleEncoder.setPositionConversionFactor(Control.arm.ENCODER_POS_UNIT_PER_DEGREE);
        angleEncoder.setVelocityConversionFactor(Control.arm.ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND);
        angleFeedforward = new ArmFeedforward(Control.arm.kAngleS, Control.arm.kAngleG, Control.arm.kAngleV, Control.arm.kAngleA);
        anglePID = new PIDController(Control.arm.kAngleP, Control.arm.kAngleI, Control.arm.kAngleD);

        armExtend = new CANSparkMax(Ports.arm.ARM_EXTENSION, MotorType.kBrushless);
        extendEncoder = armExtend.getEncoder();
        extendEncoder.setPositionConversionFactor(Control.arm.ENCODER_POS_UNIT_PER_INCH);
        extendEncoder.setVelocityConversionFactor(Control.arm.ENCODER_VEL_UNIT_PER_INCH_PER_SECOND);
        extendFeedforward = new SimpleMotorFeedforward(Control.arm.kExtendS, Control.arm.kExtendV, Control.arm.kExtendA);
        extendPID = new PIDController(Control.arm.kExtendP, Control.arm.kExtendI, Control.arm.kExtendD);
        
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

    private void angleToRotations(){

    }
    private void inchesToRotations(){

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
        setSetPoint(joystick * (angleSetPoint + 3));
    }


    @Override
    public void onLoop(){
        arm.setVoltage(
            angleFeedforward.calculate(Units.degreesToRadians(angleSetPoint), 0)
          + anglePID.calculate(arm.getEncoder().getVelocity(), angleSetPoint));
        armExtend.setVoltage(
            extendFeedforward.calculate(extensionSetPoint) 
          + extendPID.calculate(armExtend.getEncoder().getVelocity(), extensionSetPoint));
        receiveOptions();
        submitTelemetry();
    }
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
