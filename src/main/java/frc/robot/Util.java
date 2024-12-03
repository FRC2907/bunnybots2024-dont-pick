package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Control;
import edu.wpi.first.wpilibj.PS5Controller;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Util {
    public static CANSparkMax createSparkGroup(int[] ids, boolean invWhole, boolean invIndividual) {
		if (ids.length == 0) {
			System.err.println("[EE] Attempted to create empty group of CANSparkMax");
			new Exception().printStackTrace();
		}
		CANSparkMax[] mcs = new CANSparkMax[ids.length];
		for (int i = 0; i < ids.length; i++) {
			mcs[i] = new CANSparkMax(ids[i], MotorType.kBrushless);
			if (i > 0)
				mcs[i].follow(mcs[0]);
        mcs[i].setInverted(invIndividual);
		}
		mcs[0].setInverted(invWhole);
		return mcs[0];
    }

    public static double clamp(double min, double value, double max) {
        if (max < min) { 
            System.err.println("[EE] I was asked to clamp value " + value + " between min " + min + " and max " + max);
            new Exception().printStackTrace();
        }
        if (value < min) return min;
        if (max < value) return max;
        return value;
    }

    public static boolean checkDriverDeadband(double value){
        return Math.abs(value) > Control.kDriverDeadband;
    }
    public static double getLeftMagnitude(PS5Controller input){
        return Math.sqrt(Math.pow(input.getLeftX(), 2.0) + Math.pow(input.getLeftY(), 2.0));
    }
    public static double getRightMagnitude(PS5Controller input){
        return Math.sqrt(Math.pow(input.getRightX(), 2.0) + Math.pow(input.getRightY(), 2.0));
    }

    public static boolean isBlue(){
		if (DriverStation.getAlliance().isPresent()){
      	    return DriverStation.getAlliance().get() == Alliance.Blue;
		}
        System.err.println("[EE] I could not find an alliance");
        return true;
	}
    public static boolean isRed(){
		if (DriverStation.getAlliance().isPresent()){
      	    return DriverStation.getAlliance().get() == Alliance.Red;
		}
        System.err.println("[EE] I could not find an alliance");
		return true;
	}

    public static double metersPerSecondToRPM(double speed, double diameter){
        return (speed / (diameter * Math.PI)) * 60;
    }
}
