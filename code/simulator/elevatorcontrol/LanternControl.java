/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
LanternControl
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;

/* Author: James Sakai (jssakai) */
public class LanternControl extends Controller {

	/**
	 * ************************************************************************
	 * Declarations
	 * ************************************************************************
	 */

	/*
	 * OOUTPUT INTERFACE
	 */

	// Output Interface :: local physical state
	private WriteableCarLanternPayload localCarLantern;

	// Networked Output Interface
	private WriteableCanMailbox networkCarLantern;

	/*
	 * INPUT INTERFACE
	 */

	// Networked Input Interface :: received door closed message
	// private ReadableCanMailbox networkDoorClosedLeft;
	// private ReadableCanMailbox networkDoorClosedRight;
	private DoorClosedArray mDoorClosedFrontHallway;
	private DoorClosedArray mDoorClosedBackHallway;

	// Networked Input Interface :: translator for the AtFloor message --
	// this translator is specific to this message and is provided in the
	// elevatormodules package
	private AtFloorArray mAtFloor;

	// Networked Input Interface :: translator for the DesiredFloor message --
	// this translator is specific to this message and is provided in the
	// elevatormodules package
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	// these variables keep track of which instance this is.
	// private final Hallway hallway;
	private final Direction direction;
	private Direction desiredDirection;
	// private final int floor;

	// store the period for the controller
	private SimTime period;

	// enumerate states
	private enum State {
		STATE_NO_DIRECTION, STATE_DIRECTION_SET, STATE_LANTERN_ON, STATE_Lantern_NOT_ON
	}

	// initialized state
	private State currentState = State.STATE_NO_DIRECTION;

	/*
	 * The arguments listed in the .cf configuration file should match the order
	 * and type given here.
	 * 
	 * For your elevator controllers, you should make sure that the constructor
	 * matches the method signatures in ControllerBuilder.makeAll().
	 */

	public LanternControl(Direction direction, SimTime period, boolean verbose) {

		super("LanternControl"
				+ ReplicationComputer.makeReplicationString(direction), verbose);

		// store the constructor arguments in internal state
		this.period = period;
		// this.hallway = hallway;
		this.direction = direction;
		// this.floor = floor;

		/*
		 * The log() method is inherited from the Controller class. It takes an
		 * array of objects which will be converted to strings and concatenated
		 * only if the log message is actually written.
		 */
		log("Created " + getReplicationName() + " with period = ", period);

		/*
		 * Create READABLE PAYLOADS AND TRANSLATORS for all messages in INPUT
		 * INTERFACE - mAtFloor[f,b,d], mDesiredFloor, mDoorClosed[b,r]
		 */

		// mAtFloor
		mAtFloor = new AtFloorArray(canInterface);

		// mDesiredFloor
		networkDesiredFloor = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(
				networkDesiredFloor);

		// mDoorClosedLeft, mDoorClosedRight
		mDoorClosedFrontHallway = new DoorClosedArray(Hallway.FRONT,
				canInterface);
		mDoorClosedBackHallway = new DoorClosedArray(Hallway.BACK, canInterface);

		// Read input messages periodically :: TIME-TRIGGERED
		canInterface.registerTimeTriggered(networkDesiredFloor);

		/*
		 * Create WRITEABLE PAYLOADS for all messages in OUTPUT INTERFACE -
		 * HallLight[f,b,d], mHallLight[f,b,d], mHallCall[f,b,d]
		 */

		// CarLantern
		localCarLantern = CarLanternPayload.getWriteablePayload(direction);

		// Broadcast output messages periodically :: TIME-TRIGGERED
		physicalInterface.sendTimeTriggered(localCarLantern, period);

		// Start the periodic timer
		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		log("current state is " + currentState);
		State newState = currentState;

		//System.out.println("state" + currentState + "  " + desiredDirection);
		switch (currentState) {
		case STATE_NO_DIRECTION:
			localCarLantern.set(false);
			desiredDirection = Direction.STOP;
			// #transition TL.T.1
			if ((mDesiredFloor.getDirection() != desiredDirection)
					&& (desiredDirection == Direction.STOP)) {
				newState = State.STATE_DIRECTION_SET;
			}

			break;

		case STATE_DIRECTION_SET:
			localCarLantern.set(false);
			desiredDirection = mDesiredFloor.getDirection();

			if (desiredDirection == Direction.STOP) {
				newState = State.STATE_NO_DIRECTION;
				break;
			}

			// #transition TL.T.2
			if (mAtFloor.getCurrentFloor() != -1
					&& (!mDoorClosedBackHallway.getBothClosed() || !mDoorClosedFrontHallway
							.getBothClosed())) {
				if ((desiredDirection == direction)) {
					newState = State.STATE_LANTERN_ON;
					// #transition TL.T.4
				} else {
					newState = State.STATE_Lantern_NOT_ON;
				}
			}
			break;

		case STATE_LANTERN_ON:
			localCarLantern.set(true);
			// #transition TL.T.3
			if ((mDoorClosedFrontHallway.getBothClosed() && mDoorClosedBackHallway
					.getBothClosed())) {
				newState = State.STATE_NO_DIRECTION;
			}

			break;

		case STATE_Lantern_NOT_ON:
			localCarLantern.set(false);

			if ((mDoorClosedFrontHallway.getBothClosed() && mDoorClosedBackHallway
					.getBothClosed())) {
				newState = State.STATE_NO_DIRECTION;
			}
			break;
		default:
			throw new RuntimeException("State " + currentState
					+ " was not recognized.");
		}

		if (newState != currentState) {
			log(currentState.toString() + " -> " + newState.toString());
			// System.out.println(currentState.toString() + " -> " +
			// newState.toString());
		}
		currentState = newState;

		timer.start(period);
	}

	private String getReplicationName() {
		return "LanternControl"
				+ ReplicationComputer.makeReplicationString(direction);
	}

}
