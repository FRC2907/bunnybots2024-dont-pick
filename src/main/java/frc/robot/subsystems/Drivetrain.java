package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain implements ISubsystem{
    CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    MecanumDrive dt;
    MecanumDriveWheelPositions wheelPositions;

    public Drivetrain(){
        frontLeftMotor = new CANSparkMax(Ports.drivetrain.FRONT_LEFT, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(Ports.drivetrain.REAR_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(Ports.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(Ports.drivetrain.REAR_RIGHT, MotorType.kBrushless);
        dt = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        wheelPositions = new MecanumDriveWheelPositions(frontLeftMotor.getEncoder().getPosition(),
            frontRightMotor.getEncoder().getPosition(), rearLeftMotor.getEncoder().getPosition(), 
            rearRightMotor.getEncoder().getPosition());
    }

    public static Drivetrain instance; 

    public static Drivetrain getInstance(){
        if (instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }

    public void drive(double driverX, double driverY, double driverRotation){
        dt.driveCartesian(driverX, driverY, driverRotation);
    }

    public void setLocalDriveInputs(double xSpeed, double ySpeed, double zRotation){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
        MecanumDriveWheelSpeeds wheelSpeeds = (Control.drivetrain.DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds));

        frontLeftMotor.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        frontRightMotor.getPIDController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
        rearLeftMotor.getPIDController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        rearRightMotor.getPIDController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
    }
    
    @Override
    public void onLoop(){}
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
