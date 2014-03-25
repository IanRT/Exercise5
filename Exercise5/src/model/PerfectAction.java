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

		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
				
				// make sure to respect obstructed grid points
				if (!_to.isObstructed(x, y)) {

					// the action model should work out all of the different
					// ways (x,y) in the _to grid could've been reached based on
					// the _from grid and the move taken (in this case
					// HEADING.PLUS_X)

					// for example if the only way to have got to _to (x,y) was
					// from _from (x-1, y) (i.e. there was a PLUS_X move from
					// (x-1, y) then you write that to the (x, y) value

					// The below code does not move the value, just copies
					// it to the same position

					// position before move
					float fromProb =0;
					int fromX = x;
					int fromY = y;
					
					if(_from.isValidGridPoint(fromX, fromY)){
						fromProb = _from.getProbability(fromX, fromY);
					}
					
					// position after move
					int toX = x+1;
					int toY = y;
					
					
					if(!_to.isValidTransition(fromX, fromY, toX, toY))
					{
						_to.setProbability(fromX, fromY, fromProb+_from.getProbability(fromX, fromY));
					}
					else
					{
						_to.setProbability(toX, toY, fromProb);
						if(!_to.isValidTransition(fromX-1, fromY, fromX, fromY))
							_to.setProbability(fromX, fromY, 0);
					}
				}
			}
		}
	}
	private void movePlusY(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
				
				// make sure to respect obstructed grid points
				if (!_to.isObstructed(x, y)) {

					// the action model should work out all of the different
					// ways (x,y) in the _to grid could've been reached based on
					// the _from grid and the move taken (in this case
					// HEADING.PLUS_X)

					// for example if the only way to have got to _to (x,y) was
					// from _from (x-1, y) (i.e. there was a PLUS_X move from
					// (x-1, y) then you write that to the (x, y) value

					// The below code does not move the value, just copies
					// it to the same position

					// position before move
					int fromX = x;
					int fromY = y-1;
					float fromProb = _from.getProbability(fromX, fromY);

					// position after move
					int toX = x;
					int toY = y;

					if(_to.isObstructed(toX, toY+1))
						_to.setProbability(toX, toY, fromProb+_from.getProbability(toX, toY));
					else
						_to.setProbability(toX, toY, fromProb);

				}
			}
		}
	}
	private void moveMinusX(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
				
				// make sure to respect obstructed grid points
				if (!_to.isObstructed(x, y)) {

					// the action model should work out all of the different
					// ways (x,y) in the _to grid could've been reached based on
					// the _from grid and the move taken (in this case
					// HEADING.PLUS_X)

					// for example if the only way to have got to _to (x,y) was
					// from _from (x-1, y) (i.e. there was a PLUS_X move from
					// (x-1, y) then you write that to the (x, y) value

					// The below code does not move the value, just copies
					// it to the same position

					// position before move
					int fromX = x+1;
					int fromY = y;
					float fromProb = _from.getProbability(fromX, fromY);

					// position after move
					int toX = x;
					int toY = y;

					if(_to.isObstructed(toX-1, toY))
						_to.setProbability(toX, toY, fromProb+_from.getProbability(toX, toY));
					else
						_to.setProbability(toX, toY, fromProb);

				}
			}
		}
	}
	private void moveMinusY(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
				
				// make sure to respect obstructed grid points
				if (!_to.isObstructed(x, y)) {

					// the action model should work out all of the different
					// ways (x,y) in the _to grid could've been reached based on
					// the _from grid and the move taken (in this case
					// HEADING.PLUS_X)

					// for example if the only way to have got to _to (x,y) was
					// from _from (x-1, y) (i.e. there was a PLUS_X move from
					// (x-1, y) then you write that to the (x, y) value

					// The below code does not move the value, just copies
					// it to the same position

					// position before move
					int fromX = x;
					int fromY = y+1;
					float fromProb = _from.getProbability(fromX, fromY);

					// position after move
					int toX = x;
					int toY = y;

					if(_to.isObstructed(toX, toY-1))
						_to.setProbability(toX, toY, fromProb+_from.getProbability(toX, toY));
					else
						_to.setProbability(toX, toY, fromProb);

				}
			}
		}
	}
}