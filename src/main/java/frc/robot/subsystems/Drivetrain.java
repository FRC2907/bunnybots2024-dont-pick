package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Util;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain implements ISubsystem{
    private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    private RelativeEncoder flEncoder, rlEncoder, frEncoder, rrEncoder;
    private double frontLeftSetPoint, rearLeftSetPoint, frontRightSetPoint, rearRightSetPoint;
    private MecanumDrive dt;
    private MecanumDriveWheelPositions wheelPositions;
    private AHRS gyro;
    private SimpleMotorFeedforward flFeedforward, rlFeedforward, frFeedforward, rrFeedforward;
    private PIDController flPID, rlPID, frPID, rrPID;


    public Drivetrain(){
        frontLeftMotor = new CANSparkMax(Ports.drivetrain.FRONT_LEFT, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(Ports.drivetrain.REAR_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(Ports.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(Ports.drivetrain.REAR_RIGHT, MotorType.kBrushless);

        flEncoder = frontLeftMotor.getEncoder();
        rlEncoder = rearLeftMotor.getEncoder();
        frEncoder = frontRightMotor.getEncoder();
        rrEncoder = rearRightMotor.getEncoder();

        dt = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

        wheelPositions = new MecanumDriveWheelPositions(
            flEncoder.getPosition(), frEncoder.getPosition(), 
            rlEncoder.getPosition(), rrEncoder.getPosition());
        gyro = new AHRS(SPI.Port.kMXP);

        flFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);
        rlFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);
        frFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);
        rrFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);

        flPID = new PIDController(Control.drivetrain.kflP, Control.drivetrain.kflI, Control.drivetrain.kflD);
        rlPID = new PIDController(Control.drivetrain.krlP, Control.drivetrain.krlI, Control.drivetrain.krlD);
        frPID = new PIDController(Control.drivetrain.kfrP, Control.drivetrain.kfrI, Control.drivetrain.kfrD);
        rrPID = new PIDController(Control.drivetrain.krrP, Control.drivetrain.krrI, Control.drivetrain.krrD);
        }

    private static Drivetrain instance; 

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

        setWheelSpeeds(
            wheelSpeeds.frontLeftMetersPerSecond, 
            wheelSpeeds.rearLeftMetersPerSecond, 
            wheelSpeeds.frontRightMetersPerSecond,
            wheelSpeeds.rearRightMetersPerSecond);
    }

    public void setFieldDriveInputs(double xSpeed, double ySpeed, double zRotation){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, gyro.getRotation2d());
        MecanumDriveWheelSpeeds wheelSpeeds = (Control.drivetrain.DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds));

        setWheelSpeeds(
            wheelSpeeds.frontLeftMetersPerSecond, 
            wheelSpeeds.rearLeftMetersPerSecond, 
            wheelSpeeds.frontRightMetersPerSecond,
            wheelSpeeds.rearRightMetersPerSecond);
    }
    private void setWheelSpeeds(double flSetPoint, double rlSetPoint, double frSetPoint, double rrSetPoint){
        frontLeftSetPoint = flSetPoint;
        rearLeftSetPoint = rlSetPoint;
        frontRightSetPoint = frSetPoint;
        rearRightSetPoint = rrSetPoint;
    }

    private void conversion(){
        frontLeftSetPoint =  Util.metersPerSecondToRPM(frontLeftSetPoint, Units.inchesToMeters(Control.drivetrain.WHEEL_DIAMETER) * Control.drivetrain.GEAR_RATIO);
        rearLeftSetPoint =   Util.metersPerSecondToRPM(rearLeftSetPoint, Units.inchesToMeters(Control.drivetrain.WHEEL_DIAMETER) * Control.drivetrain.GEAR_RATIO);
        frontRightSetPoint = Util.metersPerSecondToRPM(frontRightSetPoint, Units.inchesToMeters(Control.drivetrain.WHEEL_DIAMETER) * Control.drivetrain.GEAR_RATIO);
        rearRightSetPoint =  Util.metersPerSecondToRPM(frontLeftSetPoint, Units.inchesToMeters(Control.drivetrain.WHEEL_DIAMETER) * Control.drivetrain.GEAR_RATIO);
    }

    
    @Override
    public void onLoop(){
        conversion();

        frontLeftMotor. setVoltage(Util.clamp(Control.MIN_VOLTAGE, 
             flFeedforward.calculate(frontLeftSetPoint)
          +  flPID.calculate(flEncoder.getVelocity(), frontLeftSetPoint), Control.MAX_VOLTAGE));
        rearLeftMotor.  setVoltage(Util.clamp(Control.MIN_VOLTAGE,
              rlFeedforward.calculate(rearLeftSetPoint)
          +   rlPID.calculate(rlEncoder.getVelocity(), rearLeftSetPoint), Control.MAX_VOLTAGE));
        frontRightMotor.setVoltage(Util.clamp(Control.MIN_VOLTAGE, 
            frFeedforward.calculate(frontRightSetPoint)
          + frPID.calculate(frEncoder.getVelocity(), frontRightSetPoint), Control.MAX_VOLTAGE));
        rearRightMotor. setVoltage(Util.clamp(Control.MIN_VOLTAGE,
             rrFeedforward.calculate(rearRightSetPoint) 
          +  rrPID.calculate(rrEncoder.getVelocity(), rearRightSetPoint), Control.MAX_VOLTAGE));
    }

    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
