/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
*/

package simulator.elevatorcontrol;

import jSimPack.SimTime;

import java.util.ArrayList;

import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * The class that represents the DriveControl object
 * 
 * @author weisiyu
 * 
 *         18649 2014 Fall Group 5 Yurui Zhou (yuruiz) Siyur Wei (siyuwei) James
 *         Sakai (jssakai) Vijay Jayaram (vijayj)
 */
public class DriveControl extends Controller {

	private static final String NAME = "DriveControl";

	private Direction desiredDirection;
	private SimTime period;

	private State currentState = State.WAIT;

	/*
	 * Input interfaces
	 */
	private BooleanCanPayloadTranslator mEmergencyBrake;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	private LevelingCanPayloadTranslator mLevelUp, mLevelDown;
	private CarWeightCanPayloadTranslator mCarWeight;

	private ReadableCanMailbox emergencyBrake;
	private ReadableCanMailbox carWeight;
	private ReadableCanMailbox levelUp, levelDown;
	private ReadableCanMailbox desiredFloor;

	private DoorClosedArray doorClosedFront, doorClosedBack;
	private AtFloorArray atFloor;
	private int currentFloor = 1;

	/*
	 * Out put interfaces
	 */

	private WriteableCanMailbox drive;
	private DriveCommandCanPayloadTranslator mDrive;

	private WriteableCanMailbox driveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;

	private WriteableDrivePayload drivePayload;

	private enum State {
		WAIT, MOVE, LEVEL, OPEN, EMERGENCY
	}

	public DriveControl(SimTime period, boolean verbose) {
		super(NAME, verbose);
		this.period = period;
		this.initialize();
	}

	private void initialize() {
		/*
		 * Inputs
		 */

		new ArrayList<ReadableCanMailbox>();

		doorClosedFront = new DoorClosedArray(Hallway.FRONT, canInterface);
		doorClosedBack = new DoorClosedArray(Hallway.BACK, canInterface);

		atFloor = new AtFloorArray(canInterface);

		/*
		 * Get all the mAtFloor messages
		 */

		/*
		 * Emergency brake message
		 */
		emergencyBrake = CanMailbox
				.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mEmergencyBrake = new BooleanCanPayloadTranslator(emergencyBrake);
		canInterface.registerTimeTriggered(emergencyBrake);

		/*
		 * Car weight alarm
		 */
		carWeight = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(carWeight);
		canInterface.registerTimeTriggered(carWeight);

		/*
		 * mDesiredDirection
		 */
		desiredFloor = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(desiredFloor);
		canInterface.registerTimeTriggered(desiredFloor);

		/*
		 * mLevel
		 */
		levelUp = CanMailbox
				.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Direction.UP));
		mLevelUp = new LevelingCanPayloadTranslator(levelUp, Direction.UP);
		levelDown = CanMailbox
				.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Direction.DOWN));
		mLevelDown = new LevelingCanPayloadTranslator(levelDown, Direction.DOWN);
		canInterface.registerTimeTriggered(levelUp);
		canInterface.registerTimeTriggered(levelDown);

		/*
		 * Outputs
		 */

		// Drive
		drivePayload = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(drivePayload, period);

		// mDrive
		drive = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
		mDrive = new DriveCommandCanPayloadTranslator(drive);
		canInterface.sendTimeTriggered(drive, period);

		// mDriveSpeed
		driveSpeed = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(driveSpeed);
		canInterface.sendTimeTriggered(driveSpeed, period);

		timer.start(period);

	}

	@Override
	public void timerExpired(Object callbackData) {

		desiredDirection = mDesiredFloor.getDirection();

		boolean allClosed = doorClosedFront.getBothClosed()
				&& doorClosedBack.getBothClosed();

		State newState = currentState;
		switch (currentState) {
		case WAIT:
			this.setOutput(Speed.STOP, desiredDirection);
			// #transition 'DC.5'
			if (this.isEmergencyCondition()) {
				newState = State.EMERGENCY;
				break;
			}

			// #transition 'DC.7'
			if(!mLevelUp.getValue() || !mLevelDown.getValue()){
				newState = State.LEVEL;
			}

			// #transition 'DC.1'
			if (allClosed && currentFloor != mDesiredFloor.getFloor()) {
				newState = State.MOVE;
			}
			break;
		case MOVE:

			if (currentFloor < mDesiredFloor.getFloor()) {
				desiredDirection = Direction.UP;
			} else {
				if (currentFloor > mDesiredFloor.getFloor())
					desiredDirection = Direction.DOWN;
				else
					desiredDirection = Direction.STOP;
			}

			this.setOutput(Speed.SLOW, desiredDirection);
			// #transition 'DC.5'
			if (this.isEmergencyCondition()) {
				newState = State.EMERGENCY;
				break;
			}
			// #transition 'DC.2'
			if (mDesiredFloor.getHallway() == Hallway.NONE) {
				break;
			}

			if (mDesiredFloor.getHallway() == Hallway.BOTH) {
				if (atFloor.isAtFloor(mDesiredFloor.getFloor(), Hallway.FRONT)) {
					newState = State.LEVEL;
				}
			} else {
				if (atFloor.isAtFloor(mDesiredFloor.getFloor(),
						mDesiredFloor.getHallway())) {
					newState = State.LEVEL;
				}
			}
			break;
		case LEVEL:
			Direction d;
			if (currentFloor < mDesiredFloor.getFloor()) {
				d = Direction.UP;
			} else {
				d = Direction.DOWN;
			}
			this.setOutput(Speed.LEVEL, d);
			// #transition 'DC.3'
			if (mLevelUp.getValue() && mLevelDown.getValue()) {
				newState = State.OPEN;
				currentFloor = atFloor.getCurrentFloor();
			}
			break;
		case OPEN:
			this.setOutput(Speed.STOP, Direction.STOP);
			// #transition 'DC.5'
			if (this.isEmergencyCondition()) {
				newState = State.EMERGENCY;
				break;
			}
			// #transition 'DC.4'
			if (!allClosed) {
				newState = State.WAIT;
			}
			break;
		case EMERGENCY:
			this.setOutput(Speed.STOP, Direction.STOP);
			// #transition 'DC.6'
			if (!this.isEmergencyCondition()) {
				newState = State.WAIT;
				break;
			}
			// #transition `DC.5`
			if(!mLevelUp.getValue() || !mLevelDown.getValue()){
				newState = State.LEVEL;
			}
			break;
		default:

			break;
		}

		currentState = newState;
		timer.start(period);

	};

	/*
	 * Helper that tests guard condition to go into emergency state. Either: -
	 * Car is overweight - mEmergencyBrake has been set to true
	 */
	private boolean isEmergencyCondition() {
		return ((this.mCarWeight.getValue() > Elevator.MaxCarCapacity) || mEmergencyBrake
				.getValue());
	}

	private void setOutput(Speed speed, Direction direction) {

		drivePayload.set(speed, direction);
		mDrive.set(speed, direction);
		switch (speed) {
		case SLOW:
			mDriveSpeed.set(1, direction);
			break;
		case STOP:
			mDriveSpeed.set(0, direction);
			break;
		case LEVEL:
			mDriveSpeed.set(1, direction);
			break;
		default:
			break;
		}
	}

}
