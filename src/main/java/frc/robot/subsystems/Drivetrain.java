package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.constants.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain implements ISubsystem{
    CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    MecanumDrive dt;

    public Drivetrain(){
        frontLeftMotor = new CANSparkMax(Ports.drivetrain.FRONT_LEFT, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(Ports.drivetrain.REAR_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(Ports.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(Ports.drivetrain.REAR_RIGHT, MotorType.kBrushless);
        dt = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }

    public static Drivetrain instance; 

    public static Drivetrain getInstance(){
        if (instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }
    
    @Override
    public void onLoop(){}
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
