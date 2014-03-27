package model;

import java.util.Random;

import lejos.nxt.Motor;
import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.GridMap;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.LocalisationUtils;

public class Local {
	
	;

	public static void main(String[] args) {
		GridMap gridMap = LocalisationUtils.create2014Map1();
		GridPositionDistribution distribution = new GridPositionDistribution(
				gridMap);
		GridFollower follow = new GridFollower();
		follow.setupPilot(56, 162, Motor.C, Motor.B);
		ActionModel actionModel = new PerfectAction();
		Random rnd = new Random();
		Heading[] dirs = new Heading[] {Heading.PLUS_X,Heading.PLUS_Y,Heading.MINUS_X, Heading.MINUS_Y}; 
		int dirCount = 0;
		Heading action = dirs[dirCount];
		
		while(true)
		{
			int rLight = follow.lSensorR.getLightValue();
			int lLight = follow.lSensorL.getLightValue();	
			if(follow.isVeeredLeft(rLight, lLight))
				follow.rightTurn();
			else if(follow.isVeeredRight(rLight, lLight))
				follow.leftTurn();
			else if(follow.atJunction(rLight, lLight))
				switch(rnd.nextInt(3))
				{
					case 0:
						follow.left90();
						if(dirCount!=0)
							dirCount--;
						else
							dirCount=4;
						action = dirs[dirCount];
						distribution = actionModel.updateAfterMove(distribution, action);
						break;
					case 1:
						follow.straightOn();
						action = dirs[dirCount];
						distribution = actionModel.updateAfterMove(distribution, action);
						break;
					case 2:
						follow.right90();
						if(dirCount!=4)
							dirCount++;
						else
							dirCount=0;
						action = dirs[dirCount];
						distribution = actionModel.updateAfterMove(distribution, action);
						break;
				}
			else
				follow.go();
			
		}

	}

}
