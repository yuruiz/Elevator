/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
RuntimeRequirementsMonitor
 */

package simulator.elevatorcontrol;

import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.CarCallArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.HallCallArray;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;

import java.lang.Override;
import java.util.HashSet;

/**
 * High level requirements runtime monitor.
 * 
 * As of Proj8, the summarize() method reports violations of: - RT-6: The Car
 * shall only stop at Floors for which there are pending calls. - RT-7: The Car
 * shall only open Doors at Hallways for which there are pending calls.
 * 
 * @author vijay
 *
 *         As of Proj10 the summarize() method reports violations of:
 *
 *         - R-T10: For each stop at a floor, at least one door reversal shall
 *         have occured before the doors are commanded to nudge.
 *
 * @author yuruiz
 * 
 *         As of Proj11 the summarize() method reports violations of:
 *
 *         R-T9: The Drive shall be commanded to fast speed to the maximum
 *         degree practicable. *
 * 
 * @author siyuwei
 */
public class RuntimeRequirementsMonitor extends RuntimeMonitor {
	DoorStateMachine doorState = new DoorStateMachine(new AtFloorArray(
			canInterface));
	DriveStateMachine driveState = new DriveStateMachine(new AtFloorArray(
			canInterface));
	SpeedStateMachine speedState = new SpeedStateMachine();
	LanternStateMachine lanternState = new LanternStateMachine();
	CarLevelPositionCanPayloadTranslator mCarPosition;
	private ReadableCanMailbox carPosition;
	private HashSet<Integer> pendingFloorCalls = new HashSet<Integer>();
	private HashSet<CallRequest> pendingDoorCalls = new HashSet<CallRequest>();

	public RuntimeRequirementsMonitor() {
		super();
		carPosition = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		canInterface.registerTimeTriggered(carPosition);
		mCarPosition = new CarLevelPositionCanPayloadTranslator(carPosition);
	}

	boolean hadPendingCall = false;
	boolean hadPendingDoorCall = false;
	private boolean pending = true;

	boolean[] hadReversal = new boolean[2];
	int totalOpeningCount = 0;
	int wastedOpeningCount = 0;
	int totalStopCount = 0;
	int wastedStopCount = 0;
	int wastedNudgeCount = 0;
	int totalNudgeCount = 0;
	int unnecessarySlow = 0;
	int incorrectOff = 0;
	int incorrectChange = 0;
	int confusedLantern = 0;

	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub

	}

	@Override
	protected String[] summarize() {
		String[] arr = new String[7];
		arr[0] = wastedStopCount + " unnecessary stops out of "
				+ totalStopCount + " total.";
		arr[1] = wastedOpeningCount + " unnecessary openings out of "
				+ totalOpeningCount + " total.";
		arr[2] = wastedNudgeCount + " unnecessary nudge out of "
				+ totalNudgeCount + " total";
		arr[3] = unnecessarySlow
				+ " times drive is commanded to slow unnecessarily";
		arr[4] = "Lantern is of while there are pending calls " + incorrectOff
				+ " times";
		arr[5] = "Lantern is lit and then changed during doors open "
				+ incorrectChange + " times";
		arr[6] = "Lantern is lit and elevator does not service that direction first "
				+ confusedLantern + " times";
		return arr;
	}

	/**************************************************************************
	 * low level message receiving methods
	 * 
	 * These mostly forward messages to the appropriate state machines
	 **************************************************************************/
	@Override
	public void receive(ReadableDoorClosedPayload msg) {
		doorState.receive(msg);
	}

	@Override
	public void receive(ReadableDoorMotorPayload msg) {
		doorState.receive(msg);
	}

	@Override
	public void receive(ReadableDrivePayload msg) {
		driveState.receive(msg);
		speedState.drive = msg.speed();
		speedState.update();
	}

	@Override
	public void receive(ReadableDriveSpeedPayload msg) {
		driveState.receive(msg);
		speedState.speed = msg.speed();
		speedState.update();

	}

	@Override
	public void receive(ReadableDoorReversalPayload msg) {
		doorState.receive(msg);
	}

	@Override
	public void receive(ReadableCarLanternPayload msg) {
		lanternState.receive(msg);
	}

	@Override
	public void receive(ReadableCarCallPayload msg) {
		driveState.receive(msg);
		doorState.receive(msg);
	}

	@Override
	public void receive(ReadableHallCallPayload msg) {
		driveState.receive(msg);
		doorState.receive(msg);
	}

	/**************************************************************************
	 * high level event methods
	 *
	 * these are called by the logic in the message receiving methods and the
	 * state machines
	 **************************************************************************/

	/**
	 * Called once when the doors close completely
	 * 
	 * @param hallway
	 *            which door the event pertains to
	 */
	private void doorClosed(Hallway hallway, int currentFloor) {
		// System.out.println(hallway.toString() + " Door Closed");
		// Once all doors are closed, check to see if opening was wasted
		if (!hadPendingDoorCall) {
			warning("Violation of R-T7: Door opened at floor " + currentFloor
					+ " and hallway " + hallway
					+ " where there were no pending calls.");
			this.wastedOpeningCount += 1;
		}
		hadPendingDoorCall = false;
		totalOpeningCount += 1;
		hadReversal[hallway.ordinal()] = false;
	}

	private void speedViolate() {
		unnecessarySlow++;
		warning("Violation of R-T9: Drive.speed is commanded as SLOW while it can be commanded as fast");
	}

	private void lanternViolate_1() {
		incorrectOff++;
		warning("Violation of R-T8.1: Lantern is not lighted while there is pending call at other floor");
	}

	private void lanternViolate_2() {
		incorrectChange++;
		warning("Violation of R-T8.2: Lantern is lit and the state is changed before door close");
	}

	private void lanternViolate_3() {
		confusedLantern++;
		warning("Violation of R-T8.3: Lantern is lit and the car does not service that direction first");
	}

	/**
	 * Called once when the doors open without a call to the floor
	 * 
	 * @param floor
	 *            which door the event pertains to
	 * @param hallway
	 *            which door the event pertains to
	 */
	private void noCallDoorOpened(int floor, Hallway hallway) {
		hadPendingDoorCall = false;
	}

	/**
	 * Called once when the doors open with a call to the floor
	 * 
	 * @param floor
	 *            which door the event pertains to
	 * @param hallway
	 *            which door the event pertains to
	 */
	private void callDoorOpened(int floor, Hallway hallway) {
		pendingDoorCalls.remove(new CallRequest(floor, hallway));
		hadPendingDoorCall = true;
	}

	/**
	 * Called once when the drive is moving
	 */
	private void driveMoving(int currentFloor) {
		// Once drive starts moving again, check if stop was wasted
		if (!hadPendingCall) {
			warning("Violation of R-T6: Drive stopped at floor " + currentFloor
					+ " with no pending calls.");
			this.wastedStopCount += 1;
		}
		hadPendingCall = false;
		totalStopCount += 1;
	}

	/**
	 * Called once when the drive is stopped at a floor with no call at that
	 * floor
	 */
	private void noCallDriveStopped(int floor) {
		// System.out.println("Drive stopped at floor " + f +
		// " without a call.");
		hadPendingCall = false;
	}

	/**
	 * Called once when the drive is stopped at a floor with a call at that
	 * floor
	 */
	private void callDriveStopped(int floor) {
		pendingFloorCalls.remove(floor);
		hadPendingCall = true;
	}

	/*
	 * Called once when the door is start nudge
	 */
	private void callDoorNudge(Hallway hallway) {
		totalNudgeCount++;
		if (!hadReversal[hallway.ordinal()]) {

			warning("Violation of R-T10: Door nudge at " + hallway
					+ " with no reversal triggered.");
			wastedNudgeCount++;
		}
	}

	private void callDoorReversal(Hallway hallway) {
		hadReversal[hallway.ordinal()] = true;
	}

	private static enum DoorState {
		CLOSED, CALL_OPEN, NO_CALL_OPEN
	}

	/**
	 * Utility class for keeping track of the door state.
	 * 
	 * Also provides external methods that can be queried to determine the
	 * current door state.
	 */
	private class DoorStateMachine {

		DoorState state[] = new DoorState[2];
		AtFloorArray atFloorArray;
		boolean[] isNudging = new boolean[2];
		boolean[] isReversaling = new boolean[2];
		private boolean pending = true;

		public DoorStateMachine(AtFloorArray atFloors) {
			state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
			state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
			isNudging[Hallway.FRONT.ordinal()] = false;
			isNudging[Hallway.BACK.ordinal()] = false;
			isReversaling[Hallway.FRONT.ordinal()] = false;
			isReversaling[Hallway.BACK.ordinal()] = false;
			this.atFloorArray = atFloors;
		}

		public void receive(ReadableCarCallPayload msg) {
			pendingDoorCalls.add(new CallRequest(msg.getFloor(), msg
					.getHallway()));
			updateState(msg.getHallway());
		}

		public void receive(ReadableHallCallPayload msg) {
			pendingDoorCalls.add(new CallRequest(msg.getFloor(), msg
					.getHallway()));
			updateState(msg.getHallway());
		}

		public void receive(ReadableDoorClosedPayload msg) {
			updateState(msg.getHallway());
		}

		public void receive(ReadableDoorMotorPayload msg) {
			updateState(msg.getHallway());
			updateNudge(msg.getHallway());
		}

		public void receive(ReadableDoorReversalPayload msg) {
			Hallway h = msg.getHallway();
			if (doorReversals[h.ordinal()][Side.LEFT.ordinal()].isReversing()
					|| doorReversals[h.ordinal()][Side.RIGHT.ordinal()]
							.isReversing()) {
				if (!isReversaling[h.ordinal()]) {
					callDoorReversal(h);
					isReversaling[h.ordinal()] = true;
				}
			} else {
				isReversaling[h.ordinal()] = false;
			}
		}

		private void updateState(Hallway h) {
			DoorState previousState = state[h.ordinal()];

			DoorState newState = previousState;
			int currentFloor = atFloorArray.getCurrentFloor();
			if (currentFloor == MessageDictionary.NONE) {
				// Not at any floor, doors must be closed.
				// No other action required
				if (!allDoorsClosed(h)) {
					throw new RuntimeException(
							"Doors are open without car being at any floor.");
				}
				return;
			}

			if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
				newState = DoorState.CLOSED;
			} else if (!allDoorsClosed(h) && !hadPendingDoorCall) {
				// Doors opened, check if need to set hadPendingDoorCall
				if (pendingDoorCalls.contains(new CallRequest(currentFloor, h))) {
					newState = DoorState.CALL_OPEN;
				} else {
					newState = DoorState.NO_CALL_OPEN;
				}
			}

			if (newState != previousState) {
				switch (newState) {
				case CLOSED:
					doorClosed(h, currentFloor);
					break;
				case CALL_OPEN:
					// System.out.println("Door opened in response to call");
					callDoorOpened(currentFloor, h);
					break;
				case NO_CALL_OPEN:
					noCallDoorOpened(currentFloor, h);
					break;

				}
			}

			// set the newState
			state[h.ordinal()] = newState;
		}

		public void updateNudge(Hallway hallway) {
			if (doorNudge(hallway)) {
				if (!isNudging[hallway.ordinal()]) {
					callDoorNudge(hallway);
					isNudging[hallway.ordinal()] = true;
				}
			} else {
				isNudging[hallway.ordinal()] = false;
			}
		}

		// door utility methods
		public boolean allDoorsClosed(Hallway h) {
			return (doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed() && doorCloseds[h
					.ordinal()][Side.RIGHT.ordinal()].isClosed());
		}

		public boolean allDoorMotorsStopped(Hallway h) {
			return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP
					&& doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
		}

		// Returns whether there a car or hall call to this floor/hallway
		// combination
		public boolean wasCalled(int f, Hallway h) {
			return carCalls[f - 1][h.ordinal()].pressed()
					|| hallCalls[f - 1][h.ordinal()][Direction.UP.ordinal()]
							.pressed()
					|| hallCalls[f - 1][h.ordinal()][Direction.DOWN.ordinal()]
							.pressed();
		}

		public boolean doorNudge(Hallway hallway) {
			return doorMotors[hallway.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE
					&& doorMotors[hallway.ordinal()][Side.RIGHT.ordinal()]
							.command() == DoorCommand.NUDGE;
		}

	}

	private static enum DriveState {
		MOVING, CALL_STOP, NO_CALL_STOP
	}

	/**
	 * Utility class for keeping track of the drive state.
	 * 
	 * Also provides external methods that can be queried to determine the
	 * current drive state.
	 */
	private class DriveStateMachine {

		DriveState currentState;
		AtFloorArray atFloorArray;
		private int lastSeenFloor;
		private boolean pending = true;

		public DriveStateMachine(AtFloorArray atFloors) {
			this.currentState = DriveState.CALL_STOP;
			this.atFloorArray = atFloors;
			this.lastSeenFloor = 1;
		}

		public void receive(ReadableDrivePayload msg) {
			updateState();
		}

		public void receive(ReadableDriveSpeedPayload msg) {
			updateState();
		}

		public void receive(ReadableCarCallPayload msg) {
			pendingFloorCalls.add(msg.getFloor());
			updateState();
		}

		public void receive(ReadableHallCallPayload msg) {
			pendingFloorCalls.add(msg.getFloor());
			updateState();
		}

		private void updateState() {
			DriveState newState = this.currentState;

			int currentFloor = atFloorArray.getCurrentFloor();
			if (currentFloor != MessageDictionary.NONE) {
				lastSeenFloor = currentFloor;
			}

			if (currentFloor == MessageDictionary.NONE) {
				newState = DriveState.MOVING;
			} else if (!hadPendingCall) {
				// Drive is stopped, check whether there was a call here or not
				if (pendingFloorCalls.contains(currentFloor)) {
					newState = DriveState.CALL_STOP;
				} else {
					newState = DriveState.NO_CALL_STOP;
				}
			}

			if (newState != this.currentState) {
				switch (newState) {
				case MOVING:
					driveMoving(lastSeenFloor);
					break;
				case CALL_STOP:
					callDriveStopped(lastSeenFloor);
					break;
				case NO_CALL_STOP:
					noCallDriveStopped(lastSeenFloor);
					break;
				}
			}

			// set the newState
			this.currentState = newState;
		}

		// drive utility methods
		public boolean driveStopped() {
			return Speed.isStopOrLevel(driveCommandedSpeed.speed());
		}

		public void printCalled() {
			for (int f = 1; f < 8; f++) {
				if (wasCalled(f, Hallway.FRONT, Direction.UP))
					System.out.println(f + " FRONT UP");
				if (wasCalled(f, Hallway.FRONT, Direction.DOWN))
					System.out.println(f + " FRONT DOWN:");
				if (wasCalled(f, Hallway.BACK, Direction.UP))
					System.out.println(f + " BACK UP:");
				if (wasCalled(f, Hallway.BACK, Direction.DOWN))
					System.out.println(f + " BACK DOWN:");
			}
		}

		// Checks whether calls were made from either hallway
		public boolean wasCalled(int f) {
			return (wasCalled(f, Hallway.BACK) || wasCalled(f, Hallway.FRONT));
		}

		// Returns whether there a car or hall call to this floor/hallway
		// combination
		public boolean wasCalled(int f, Hallway h) {
			return carCalls[f - 1][h.ordinal()].pressed()
					|| hallCalls[f - 1][h.ordinal()][Direction.UP.ordinal()]
							.pressed()
					|| hallCalls[f - 1][h.ordinal()][Direction.DOWN.ordinal()]
							.pressed();
		}

		private boolean wasCalled(int f, Hallway h, Direction d) {
			return hallCalls[f - 1][h.ordinal()][d.ordinal()].pressed();
		}
	}

	private enum SpeedState {
		FAST, SLOW, SLOW_F
	}

	private class SpeedStateMachine {
		private SpeedState currState = SpeedState.SLOW;
		private double speed = -1;
		private int position = 0;
		private int desiredFloor = -1;
		private Speed drive;
		private static final double RESIDUAL = 700;
		private int count = 0;

		/**
		 * Helper method for deciding if fast is possible
		 * 
		 * @return
		 */
		private boolean fastAvailable() {
			int desiredPosition = (desiredFloor - 1) * 5000;
			double stopDist = Math.pow(speed * 1000, 2)
					/ (2 * DriveObject.Acceleration * 1000);

			return ((position < desiredPosition && position + stopDist
					+ RESIDUAL < desiredPosition) || (position > desiredPosition && position
					- stopDist - RESIDUAL > desiredPosition))
					&& speed > DriveObject.SlowSpeed;
		}

		/**
		 * helper method tells that if all information has been received at
		 * least once
		 */
		private boolean initialized() {
			return this.speed >= 0 && this.desiredFloor >= 0;
		}

		private void update() {
			desiredFloor = mDesiredFloor.getFloor();
			position = mCarPosition.getPosition();

			if (!initialized()) {
				return;
			}

			SpeedState newState = currState;
			switch (currState) {
			case SLOW:
				if (this.fastAvailable() && drive != Speed.FAST) {
					count = 0;
					newState = SpeedState.SLOW_F;
				}
				if (drive == Speed.FAST) {
					newState = SpeedState.FAST;
				}
				break;
			case SLOW_F:
				count++;
				if (count == 12) {
					speedViolate();
				}
				if (drive == Speed.FAST) {
					newState = SpeedState.FAST;
					break;
				}
				if (!this.fastAvailable()) {
					newState = SpeedState.SLOW;
				}
				break;
			case FAST:
				if (drive != Speed.FAST) {
					if (!this.fastAvailable()) {
						newState = SpeedState.SLOW;
					} else {
						count = 0;
						newState = SpeedState.SLOW_F;
					}
				}
			}
			currState = newState;
		}
	}

	private enum LanternState {
		CLOSED, OFF, UP, DOWN;
	}

	private class LanternStateMachine {

		private LanternState state = LanternState.CLOSED;
		private int count = 0;

		private CarCallArray carCalls = new CarCallArray(canInterface);
		private HallCallArray hallCalls = new HallCallArray(canInterface);
		private AtFloorArray atFloor = new AtFloorArray(canInterface);
		private ReadableCarLanternPayload upLantern = null;
		private ReadableCarLanternPayload downLantern = null;
		private DoorClosedArray frontDoors = new DoorClosedArray(Hallway.FRONT,
				canInterface);
		private DoorClosedArray backDoors = new DoorClosedArray(Hallway.BACK,
				canInterface);

		public void receive(ReadableCarLanternPayload msg) {
			if (msg.getDirection() == Direction.UP) {
				upLantern = msg;
			} else {
				downLantern = msg;
			}
			update();
		}

		private boolean allClosed() {
			return frontDoors.getBothClosed() && backDoors.getBothClosed();
		}

		private boolean initialized() {
			return upLantern != null && downLantern != null;
		}

		private boolean otherPendingCall() {
			for (int i = 1; i <= Elevator.numFloors; i++) {
				if (atFloor.getCurrentFloor() == i) {
					continue;
				}
				if (carCalls.isCalled(i).isValid()
						|| hallCalls.isCalled(i).isValid()) {
					return true;
				}
			}

			return false;
		}

		public void update() {

			if (!initialized() || atFloor.getCurrentFloor() == -1) {
				return;
			}

			LanternState newState = state;
			switch (state) {
			case OFF:
				if (otherPendingCall()) {
					count++;
				}

				/*
				 * If the monitor does not light after door closed for 15
				 * periods
				 */
				if (count == 15) {
					lanternViolate_1();
				}

				if (allClosed()) {
					newState = LanternState.CLOSED;
					break;
				}
				if (upLantern.lighted()) {
					newState = LanternState.UP;
					break;
				}
				if (downLantern.lighted()) {
					newState = LanternState.DOWN;
					break;
				}
				break;
			case UP:
				if (allClosed()) {
					if (mDesiredFloor.getFloor() < atFloor.getCurrentFloor()) {
						lanternViolate_3();
					}
					newState = LanternState.CLOSED;
					break;
				}
				if (downLantern.lighted()) {
					if (!allClosed()) {
						lanternViolate_2();
					}
					newState = LanternState.DOWN;
					break;
				}
				if (!downLantern.lighted() && !upLantern.lighted()) {
					newState = LanternState.OFF;
					count = 0;
					break;
				}
				break;
			case DOWN:
				if (allClosed()) {
					if (mDesiredFloor.getFloor() > atFloor.getCurrentFloor()) {
						lanternViolate_3();
					}
					newState = LanternState.CLOSED;
					break;
				}
				if (upLantern.lighted()) {
					if (!allClosed()) {
						lanternViolate_2();
					}
					newState = LanternState.DOWN;
					break;
				}
				if (!downLantern.lighted() && !upLantern.lighted()) {
					newState = LanternState.OFF;
					count = 0;
					break;
				}
				break;
			case CLOSED:
				if (!allClosed()) {
					if (upLantern.lighted()) {
						newState = LanternState.UP;
						break;
					}
					if (downLantern.lighted()) {
						newState = LanternState.DOWN;
						break;
					}
					newState = LanternState.OFF;
					count = 0;

				}
				break;
			}
			state = newState;
		}
	}

	private class CallRequest {
		int floor;
		Hallway hallway;

		CallRequest(int floor, Hallway hallway) {
			this.floor = floor;
			this.hallway = hallway;
		}

		@Override
		public boolean equals(Object o) {
			if (!(o instanceof CallRequest)) {
				return false;
			} else {
				CallRequest other = (CallRequest) o;
				return (other.floor == this.floor)
						&& (other.hallway.equals(this.hallway));
			}
		}

		@Override
		public int hashCode() {
			return ("" + this.floor + " " + this.hallway).hashCode();
		}

	}
}
