package model;

import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.LightSensor;
import lejos.nxt.addon.OpticalDistanceSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;


public class GridFollower
{

	private DifferentialPilot pilot;
	private RegulatedMotor rMotor;
	private RegulatedMotor lMotor;
	private OpticalDistanceSensor dSensor;
	private final float blockedDist = 0;
	LightSensor lSensorR;
	LightSensor lSensorL;
	private final double turnRatioThreshold = 1.15;
	private final int gridThreshold = 44;
	
	public void setupPilot(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor)
	{
		pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor);
		rMotor = rightMotor;
		lMotor = leftMotor;
		rMotor.setSpeed((int) (rMotor.getMaxSpeed() / 2));
		lMotor.setSpeed((int) (lMotor.getMaxSpeed() / 2));
		dSensor = new OpticalDistanceSensor(SensorPort.S4);
		lSensorR = new LightSensor(SensorPort.S2);
		lSensorL = new LightSensor(SensorPort.S3);
		lSensorR.setFloodlight(true);
		lSensorL.setFloodlight(true);
		pilot.setTravelSpeed(pilot.getMaxTravelSpeed()*0.5);
		pilot.setRotateSpeed(pilot.getMaxRotateSpeed());
	}
	
	public DifferentialPilot accessPilot()
	{
		return pilot;
	}
	
	

	public void go()
	{
		pilot.forward();
	}

	public void rightTurn()// make robot turn about right light sensor
	{
		lMotor.setSpeed((int) (lMotor.getMaxSpeed() / 2));
		lMotor.forward();
		rMotor.setSpeed((int) (rMotor.getMaxSpeed() / 5));
		rMotor.backward();
	}

	public void leftTurn()// make robot turn about left light sensor
	{
		rMotor.setSpeed((int) (lMotor.getMaxSpeed() / 2));
		rMotor.forward();
		lMotor.setSpeed((int) (rMotor.getMaxSpeed() / 5));
		lMotor.backward();
	}
	
	public void straightOn()
	{
		pilot.travel(55);
	}
	
	public void left90()
	{
		pilot.travel(55);
		pilot.rotate(90);
	}
	
	public void right90()
	{
		pilot.travel(55);
		pilot.rotate(-90);
	}
	
	public boolean isBlocked()
	{
		float look1 = accessDSensor().getRange();
		Delay.msDelay(100);
		float look2 = accessDSensor().getRange();
		Delay.msDelay(100);
		float look3 = accessDSensor().getRange();
		float avr = (look1+look2+look3)/3;
		if(avr <= getBlockedDist())
			return true;
		else
			return false;
		
	}
	public OpticalDistanceSensor accessDSensor()
	{
		return dSensor;
	}

	public float getBlockedDist() {
		return blockedDist;
	}

	public double getTurnRatioThreshold() {
		return turnRatioThreshold;
	}

	public int getGridThreshold() {
		return gridThreshold;
	}
	
	public boolean atJunction(int rLight, int lLight)
	{
		if(rLight < gridThreshold && lLight < gridThreshold)
			return true;
		else
			return false;	
	}
	
	public boolean isVeeredLeft(int rLight, int lLight)
	{
		if((double) lLight / (double) rLight > turnRatioThreshold)
			return true;
		else
			return false;
	}
	
	public boolean isVeeredRight(int rLight, int lLight)
	{
		if((double) rLight / (double) lLight > turnRatioThreshold)
			return true;
		else
			return false;
	}
	
	public void move1Forward()
	{
		go();
		//if(atJunction())
		{
			
		}
	}
	
}
