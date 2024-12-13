package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain implements ISubsystem{
    private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    private RelativeEncoder flEncoder, rlEncoder, frEncoder, rrEncoder;
    private double frontLeftSetPoint, rearLeftSetPoint, frontRightSetPoint, rearRightSetPoint;
    private MecanumDrive dt;
    //private MecanumDriveWheelPositions wheelPositions;
    private AHRS gyro;
    private SimpleMotorFeedforward flFeedforward, rlFeedforward, frFeedforward, rrFeedforward;
    private PIDController flPID, rlPID, frPID, rrPID;
    private Double[] flOutput, frOutput, rlOutput, rrOutput;
    private double flVelAverage, frVelAverage, rlVelAverage, rrVelAverage;
    private int head;


    public Drivetrain(){
        frontLeftMotor = new CANSparkMax(Ports.drivetrain.FRONT_LEFT, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(Ports.drivetrain.REAR_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(Ports.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(Ports.drivetrain.REAR_RIGHT, MotorType.kBrushless);

        flEncoder = frontLeftMotor.getEncoder();
        rlEncoder = rearLeftMotor.getEncoder();
        frEncoder = frontRightMotor.getEncoder();
        rrEncoder = rearRightMotor.getEncoder();

        //dt = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

        /*wheelPositions = new MecanumDriveWheelPositions(
            flEncoder.getPosition(), frEncoder.getPosition(), 
            rlEncoder.getPosition(), rrEncoder.getPosition());*/
        gyro = new AHRS(SPI.Port.kMXP);

        flFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);
        rlFeedforward = new SimpleMotorFeedforward(Control.drivetrain.krlS, Control.drivetrain.kV, Control.drivetrain.kA); //different static gain
        frFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);
        rrFeedforward = new SimpleMotorFeedforward(Control.drivetrain.kS, Control.drivetrain.kV, Control.drivetrain.kA);

        flPID = new PIDController(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD);
        rlPID = new PIDController(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD);
        frPID = new PIDController(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD);
        rrPID = new PIDController(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD);

        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        rearLeftMotor.setInverted(false);
        rearRightMotor.setInverted(true);

        flEncoder.setVelocityConversionFactor(1);
        frEncoder.setVelocityConversionFactor(1);
        rlEncoder.setVelocityConversionFactor(1);
        rrEncoder.setVelocityConversionFactor(1);

        flEncoder.setPositionConversionFactor(1);
        frEncoder.setPositionConversionFactor(1);
        rlEncoder.setPositionConversionFactor(1);
        rrEncoder.setPositionConversionFactor(1);

        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        rearLeftMotor.setIdleMode(IdleMode.kBrake);
        rearRightMotor.setIdleMode(IdleMode.kBrake);

        flOutput = new Double[10];
        frOutput = new Double[10];
        rlOutput = new Double[10];
        rrOutput = new Double[10];

        gyro.reset();
        head = 0;
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

    private void gainTesting(){
        frontLeftSetPoint = 0.5;
        frontRightSetPoint = 0.5;
        rearLeftSetPoint = 0.5;
        rearRightSetPoint = 0.5;
    }

    private void outputAverage(){
       flOutput = Util.arrayReplace(flOutput, head, flEncoder.getVelocity());
       flVelAverage = Util.arrayAverage(flOutput);
       frOutput = Util.arrayReplace(frOutput, head, frEncoder.getVelocity());
       frVelAverage = Util.arrayAverage(frOutput);
       rlOutput = Util.arrayReplace(rlOutput, head, rlEncoder.getVelocity());
       rlVelAverage = Util.arrayAverage(rlOutput);
       rrOutput = Util.arrayReplace(rrOutput, head, rrEncoder.getVelocity());
       rrVelAverage = Util.arrayAverage(rrOutput);

       head++;
    }

    
    @Override
    public void onLoop(){
        gainTesting();
        outputAverage();

        frontLeftMotor.setVoltage(Util.clamp(Control.MIN_VOLTAGE, 
             flFeedforward.calculate(frontLeftSetPoint)
         +   flPID.calculate(Util.RPMToMetersPerSecond(flVelAverage, Control.drivetrain.WHEEL_DIAMETER) / Control.drivetrain.GEAR_RATIO, frontLeftSetPoint),
                                             Control.MAX_VOLTAGE));

        rearLeftMotor.setVoltage(Util.clamp(Control.MIN_VOLTAGE,
             rlFeedforward.calculate(rearLeftSetPoint)
         +   rlPID.calculate(Util.RPMToMetersPerSecond(rlVelAverage, Control.drivetrain.WHEEL_DIAMETER) / Control.drivetrain.GEAR_RATIO, rearLeftSetPoint), 
                                            Control.MAX_VOLTAGE));

        frontRightMotor.setVoltage(Util.clamp(Control.MIN_VOLTAGE, 
            frFeedforward.calculate(frontRightSetPoint)
         +  frPID.calculate(Util.RPMToMetersPerSecond(frVelAverage, Control.drivetrain.WHEEL_DIAMETER) / Control.drivetrain.GEAR_RATIO, frontRightSetPoint), 
                                              Control.MAX_VOLTAGE));

        rearRightMotor.setVoltage(Util.clamp(Control.MIN_VOLTAGE,
            rrFeedforward.calculate(rearRightSetPoint) 
         +  rrPID.calculate(Util.RPMToMetersPerSecond(rrVelAverage, Control.drivetrain.WHEEL_DIAMETER) / Control.drivetrain.GEAR_RATIO, rearRightSetPoint), 
                                             Control.MAX_VOLTAGE));
        
        submitTelemetry();
        receiveOptions();
    }

    @Override
    public void submitTelemetry(){
        SmartDashboard.putNumber("dt_flVelocity", flEncoder.getVelocity() / 5.95);
        SmartDashboard.putNumber("dt_frVelocity", frEncoder.getVelocity() / 5.95);
        SmartDashboard.putNumber("dt_rlVelocity", rlEncoder.getVelocity() / 5.95);
        SmartDashboard.putNumber("dt_rrVelocity", rrEncoder.getVelocity() / 5.95);
        SmartDashboard.putNumber("dt_flSetPoint", Util.metersPerSecondToRPM(frontLeftSetPoint, Units.inchesToMeters(6)));
        SmartDashboard.putNumber("dt_frSetPoint", Util.metersPerSecondToRPM(frontRightSetPoint, Units.inchesToMeters(6)));
        SmartDashboard.putNumber("dt_rlSetPoint", Util.metersPerSecondToRPM(rearLeftSetPoint, Units.inchesToMeters(6)));
        SmartDashboard.putNumber("dt_rrSetPoint", Util.metersPerSecondToRPM(rearRightSetPoint, Units.inchesToMeters(6)));
        SmartDashboard.putNumber("dt_heading", gyro.getAngle());
        SmartDashboard.putNumber("dt_angVel", gyro.getRate());
        SmartDashboard.putNumberArray("dt_flOutput", flOutput);
        SmartDashboard.putNumberArray("dt_frOutput", frOutput);
        SmartDashboard.putNumberArray("dt_rlOutput", rlOutput);
        SmartDashboard.putNumberArray("dt_rrOutput", rrOutput);
        /*SmartDashboard.putData("dt_flPID", flPID);
        SmartDashboard.putData("dt_frPID", frPID);
        SmartDashboard.putData("dt_rlPID", rlPID);
        SmartDashboard.putData("dt_rrPID", rrPID);*/
    }
    
    @Override
    public void receiveOptions(){
        /*frontLeftSetPoint = SmartDashboard.getNumber("dt_flSetpoint", 0);
        frontRightSetPoint = SmartDashboard.getNumber("dt_frSetpoint", 0);
        rearLeftSetPoint = SmartDashboard.getNumber("dt_rlSetpoint", 0);
        rearRightSetPoint = SmartDashboard.getNumber("dt_rrSetpoint", 0);*/
    }
}
