package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain implements ISubsystem{
    int frontLeftID = 1, rearLeftID = 2, frontRightID = 3, rearRightID = 4;
    CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    MecanumDrive dt;

    public Drivetrain(){
        frontLeftMotor = new CANSparkMax(frontLeftID, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(rearLeftID, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(frontRightID, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(rearRightID, MotorType.kBrushless);
        dt = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }


    
    @Override
    public void onLoop(){}
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
