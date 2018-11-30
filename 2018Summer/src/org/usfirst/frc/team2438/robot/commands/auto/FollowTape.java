package org.usfirst.frc.team2438.robot.commands.auto;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2438.robot.commands.CommandBase;
import org.usfirst.frc.team2438.robot.util.TargetCounter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableListener;

/**
 *
 */
public class FollowTape extends CommandBase {
//Board to be held above neck around 4 inches from head, try parallel
	
	NetworkTable table;
	NetworkTableInstance instance;
	NetworkTableEntry centerXEntry;
	NetworkTableEntry centerYEntry;
	NetworkTableEntry areaEntry;
	
	Number[] defaultValue;
	Number[] centerXs;
	Number[] centerYs;
	Number[] areas;
	
	int rightVelocity = 0;
	int leftVelocity = 0;
	double centerXUpper = 89; //TODO- test this
	double centerXLower = 78; //TODO- test this
	double centerYThreshold = 25; //TODO- test this
	double areaThreshold = 0;
	double centerXDouble = 0;
	double centerYDouble = 0;
	double areaDouble = 0;
	double powerDampener = .2;
	double defaultPower = -1; 
	
    public FollowTape() {
    	requires(drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	instance = NetworkTableInstance.getDefault();
    	table = instance.getTable("GRIP/myContoursReport");
    	
    	defaultValue = new Number[0];
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	centerXEntry = table.getEntry("centerX");
    	centerYEntry = table.getEntry("centerY");
    	areaEntry    = table.getEntry("area");
    	centerXs = centerXEntry.getNumberArray(defaultValue);
    	centerYs = centerYEntry.getNumberArray(defaultValue);
    	areas    = areaEntry.getNumberArray(defaultValue);
    
    	
    	for(Number centerX : centerXs)
    	{
    		centerXDouble = centerX.doubleValue();
    		SmartDashboard.putNumber("centerX", centerXDouble);
    		System.out.print("centerX: "+ centerXDouble);
    		if(centerXDouble > centerXUpper)
    		{
    			drivetrain.setTank(defaultPower, defaultPower + powerDampener);
    		}
    		else if(centerXDouble < centerXLower)
    		{
    			drivetrain.setTank(defaultPower + powerDampener, defaultPower);
    		}
    		else
    		{
    			drivetrain.setTank(defaultPower, defaultPower);
    		}
    	}
    	for(Number centerY : centerYs)
    	{
    		centerYDouble = centerY.doubleValue();
    		SmartDashboard.putNumber("centerY", centerYDouble);
    		System.out.println(", centerY: "+centerYDouble);
    		if(centerYDouble < centerYThreshold)
    		{
    			drivetrain.stop();
    		
    			/*if(centerXDouble > centerXUpper)
        		{
        			new AutoTurn(5);
        		}
        		else if(centerXDouble < centerXLower)
        		{
        			new AutoTurn(-5);
        		}
        		else
        		{
        			this.end();
        		}*/
    			if(centerXDouble < centerXUpper && centerXDouble > centerXLower)
    			{
    				this.end();
    			}
    		}
    	}
    	
    	SmartDashboard.putNumber("Power Left", drivetrain.getLeftPower());
    	SmartDashboard.putNumber("Right Power", drivetrain.getRightPower());
    	
    	Timer.delay(.01);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	drivetrain.setTank(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
