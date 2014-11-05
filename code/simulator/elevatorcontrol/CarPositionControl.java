/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
*/


package simulator.elevatorcontrol;

import java.util.ArrayList;
import java.util.List;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public class CarPositionControl extends Controller {

	private static final String NAME = "CarPositionControl";
	private static final int FLOOR = 8;
	private int currentFloor; // the first floor
	private SimTime period;
	private State state;

	// input
	private AtFloorArray atFloor;

	private static class Door {

		private Door(int floor, Hallway hallway) {
			this.floor = floor;
			this.hallway = hallway;
		}

		private int floor;
		private Hallway hallway;
	}

	private List<Door> doorList;

	// output
	private WriteableCanMailbox carPositionIndicator;
	private IntegerCanPayloadTranslator mCarPositionIndicator;

	private WriteableCarPositionIndicatorPayload carPositionPayload;

	private enum State {
		ARRIVE, MOVING
	}

	public CarPositionControl(SimTime period, boolean verbose) {
		super(NAME, verbose);
		this.period = period;
		this.initialize();
	}

	private void initialize() {

		state = State.ARRIVE;
		currentFloor = 1; // the first floor
		carPositionIndicator = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.CAR_POSITION_CAN_ID);
		canInterface.sendTimeTriggered(carPositionIndicator, period);
		

		mCarPositionIndicator = new IntegerCanPayloadTranslator(
				carPositionIndicator);

		carPositionPayload = CarPositionIndicatorPayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(carPositionPayload, period);

		doorList = new ArrayList<Door>();
		for (int i = 1; i <= FLOOR; i++) {
			if (i == 2) {
				continue;
			}
			doorList.add(new Door(i, Hallway.FRONT));
		}

		doorList.add(new Door(1, Hallway.BACK));
		doorList.add(new Door(2, Hallway.BACK));
		doorList.add(new Door(7, Hallway.BACK));

		atFloor = new AtFloorArray(canInterface);

		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {

		State newState = state;

		switch (state) {
		case MOVING:

			carPositionPayload.set(currentFloor);
			mCarPositionIndicator.set(currentFloor);

			for (Door door : doorList) {
				// #transition CPC T.1
				if (atFloor.isAtFloor(door.floor, door.hallway)) {
					newState = State.ARRIVE;
				}

			}
			break;
		case ARRIVE:
			boolean notAtFloor = true;

			for (Door door : doorList) {
				if (atFloor.isAtFloor(door.floor, door.hallway)) {
					currentFloor = door.floor;
					log("floor: " + door.floor);
					notAtFloor = false;
				}
			}
			carPositionPayload.set(currentFloor);
			mCarPositionIndicator.set(currentFloor);

			// #transition CPC T.2
			if (notAtFloor) {
				newState = State.MOVING;
			}

			break;
		}
		state = newState;
		timer.start(period);
	}
}
