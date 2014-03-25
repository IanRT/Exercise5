package model;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.Heading;

public class PerfectAction implements ActionModel {

	@Override
	public GridPositionDistribution updateAfterMove(
			GridPositionDistribution _from, Heading _heading) {

		// Create the new distribution that will result from applying the action
		// model
		GridPositionDistribution to = new GridPositionDistribution(_from);

		// Move the probability in the correct direction for the action
		if (_heading == Heading.PLUS_X) {
			movePlusX(_from, to);
		} else if (_heading == Heading.PLUS_Y) {
			movePlusY(_from, to);
		} else if (_heading == Heading.MINUS_X) {
			moveMinusX(_from, to);
		} else if (_heading == Heading.MINUS_Y) {
			moveMinusY(_from, to);
		}

		return to;
	}

	/**
	 * Move probabilities from _from one cell in the plus x direction into _to
	 * 
	 * @param _from
	 * @param _to
	 */
	private void movePlusX(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
				if (!_to.isObstructed(x, y)) {

					float fromProb =0;
					int fromX = x;
					int fromY = y;
					
					if(_from.isValidGridPoint(fromX, fromY)){
						fromProb = _from.getProbability(fromX, fromY);
					}
					
					int toX = x+1;
					int toY = y;
					
					
					if(!_to.isValidTransition(fromX, fromY, toX, toY) && _to.isValidTransition(fromX-1, fromY, fromX, fromY))
					{
						_to.setProbability(fromX, fromY, fromProb+_from.getProbability(fromX-1, fromY));
					}
					else if(_to.isValidTransition(fromX, fromY, toX, toY))
					{
						_to.setProbability(toX, toY, fromProb);
						
					}
					if(!_to.isValidTransition(fromX-1, fromY, fromX, fromY))
						_to.setProbability(fromX, fromY, 0);
				}
			}
		}
		_to.normalise();
	}
	private void movePlusY(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
				if (!_to.isObstructed(x, y)) {

					float fromProb =0;
					int fromX = x;
					int fromY = y;
					
					if(_from.isValidGridPoint(fromX, fromY)){
						fromProb = _from.getProbability(fromX, fromY);
					}
					
					int toX = x;
					int toY = y+1;
					
					
					if(!_to.isValidTransition(fromX, fromY, toX, toY) && _to.isValidTransition(fromX, fromY-1, fromX, fromY))
					{
						_to.setProbability(fromX, fromY, fromProb+_from.getProbability(fromX, fromY-1));
					}
					else if(_to.isValidTransition(fromX, fromY, toX, toY))
					{
						_to.setProbability(toX, toY, fromProb);
						
					}
					if(!_to.isValidTransition(fromX, fromY-1, fromX, fromY))
						_to.setProbability(fromX, fromY, 0);
				}
			}
		}
		_to.normalise();
	}
	private void moveMinusX(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		for (int y = _to.getGridHeight()-1; y > -1; y--) {

			for (int x = _to.getGridWidth()-1; x >-1 ; x--) {
				if (!_to.isObstructed(x, y)) {

					float fromProb =0;
					int fromX = x;
					int fromY = y;
					
					if(_from.isValidGridPoint(fromX, fromY)){
						fromProb = _from.getProbability(fromX, fromY);
					}
					
					int toX = x-1;
					int toY = y;
					
					
					if(!_to.isValidTransition(fromX, fromY, toX, toY) && _to.isValidTransition(fromX+1, fromY, fromX, fromY))
					{
						_to.setProbability(fromX, fromY, fromProb+_from.getProbability(fromX+1, fromY));
					}
					else if(_to.isValidTransition(fromX, fromY, toX, toY))
					{
						_to.setProbability(toX, toY, fromProb);
						
					}
					if(!_to.isValidTransition(fromX+1, fromY, fromX, fromY))
						_to.setProbability(fromX, fromY, 0);
				}
			}
		}
		_to.normalise();
	}
	private void moveMinusY(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		for (int y = _to.getGridHeight()-1; y > -1; y--) {

			for (int x = _to.getGridWidth()-1; x >-1 ; x--) {
				if (!_to.isObstructed(x, y)) {

					float fromProb =0;
					int fromX = x;
					int fromY = y;
					
					if(_from.isValidGridPoint(fromX, fromY)){
						fromProb = _from.getProbability(fromX, fromY);
					}
					
					int toX = x;
					int toY = y-1;
					
					
					if(!_to.isValidTransition(fromX, fromY, toX, toY) && _to.isValidTransition(fromX, fromY+1, fromX, fromY))
					{
						_to.setProbability(fromX, fromY, fromProb+_from.getProbability(fromX, fromY+1));
					}
					else if(_to.isValidTransition(fromX, fromY, toX, toY))
					{
						_to.setProbability(toX, toY, fromProb);
						
					}
					if(!_to.isValidTransition(fromX, fromY+1, fromX, fromY))
						_to.setProbability(fromX, fromY, 0);
				}
			}
		}
		_to.normalise();
	}
}