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
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.framework.*;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

import java.util.ArrayList;

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

	private State currentState = State.STOP;

	/*
	 * Input interfaces
	 */
	private BooleanCanPayloadTranslator mEmergencyBrake;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	private LevelingCanPayloadTranslator mLevelUp, mLevelDown;
	private CarWeightCanPayloadTranslator mCarWeight;
	private ReadableDriveSpeedPayload driveSpeedPayload;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;

	private ReadableCanMailbox emergencyBrake;
	private ReadableCanMailbox carWeight;
	private ReadableCanMailbox levelUp, levelDown;
	private ReadableCanMailbox desiredFloor;
	private ReadableCanMailbox carLevelPosition;

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
		STOP, FAST, SLOW, LEVEL_UP, LEVEL_DOWN, EMERGENCY
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

		// driveSpeed pay load
		driveSpeedPayload = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(driveSpeedPayload);

		/*
		 * car position pay load
		 */

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
		 * mCarLevelPosition
		 */
		carLevelPosition = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(
				carLevelPosition);
		canInterface.registerTimeTriggered(carLevelPosition);

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

		// System.out.println("D:" + mDesiredFloor.getFloor());
		// System.out.println(currentFloor);
		// System.out.println(currentState);
		/*
		 * If the elevator is ready to set new direction
		 */
		if (atFloor.getCurrentFloor() != -1
				&& driveSpeedPayload.speed() <= DriveObject.LevelingSpeed) {
			if (currentFloor < mDesiredFloor.getFloor()) {
				desiredDirection = Direction.UP;
			} else {
				if (currentFloor > mDesiredFloor.getFloor())
					desiredDirection = Direction.DOWN;
				else
					desiredDirection = Direction.STOP;
			}
		}
		/*
		 * Remain in the same direction so drive command is adjacent to speed
		 */
		else {
			desiredDirection = driveSpeedPayload.direction();
		}

		boolean allClosed = doorClosedFront.getBothClosed()
				&& doorClosedBack.getBothClosed();

		State newState = currentState;
		switch (currentState) {
		case STOP: // #State 1 STOP
			this.setOutput(Speed.STOP, Direction.STOP);

			// #transition 'DC.T.1'
			if (!mLevelUp.getValue()) {
				newState = State.LEVEL_UP;
				break;
			}

			// #transition 'DC.T.3'
			if (!mLevelDown.getValue()) {
				newState = State.LEVEL_DOWN;
				break;
			}
			// log(allClosed + " " + currentFloor + mDesiredFloor.getFloor());
			// #transition 'DC.T.5'
			if (allClosed && currentFloor != mDesiredFloor.getFloor()) {
				log("level up to slow");
				newState = State.SLOW;
			}
			break;
		case SLOW: // #State 4 SLOW
			this.setOutput(Speed.SLOW, desiredDirection);
			// #transition 'DC.T.10'
			if (this.isEmergencyCondition()) {
				newState = State.EMERGENCY;
				break;
			}

			// System.out.println(atFloor.getCurrentFloor() + " " +
			// driveSpeedPayload.speed());
			if (atFloor.getCurrentFloor() == mDesiredFloor.getFloor()
					&& driveSpeedPayload.speed() <= DriveObject.SlowSpeed) {
				// #transition `DC.T.6`
				if (!mLevelUp.getValue()) {
					newState = State.LEVEL_UP;
				}
				// #transition `DC.T.7`
				else {
					log("slow to level down");
					newState = State.LEVEL_DOWN;
				}
				break;
			}
			// #transition `DC.T.8`
			if (!this.commitPointReached()
					&& (int) (driveSpeedPayload.speed() * 100) >= (int) (DriveObject.SlowSpeed * 100)) {
				log("go fast");
				newState = State.FAST;
			}
			break;
		case LEVEL_UP: // #State 2 LEVEL UP
			this.setOutput(Speed.LEVEL, Direction.UP);
			// #transition 'DC.T.2'
			if (mLevelUp.getValue()) {
				System.out.println("level up to stop");
				newState = State.STOP;
				currentFloor = atFloor.getCurrentFloor();
			}
			break;
		case LEVEL_DOWN: // #State 3 LEVEL DOWN
			this.setOutput(Speed.LEVEL, Direction.DOWN);
			// #transition `DC.T.4`
			if (mLevelDown.getValue()) {
				log("level down to stop");
				newState = State.STOP;
				currentFloor = atFloor.getCurrentFloor();
			}
			break;
		case FAST: // #State 5 FAST
			this.setOutput(Speed.FAST, desiredDirection);
			log(this.commitPointReached());
			// #transition `DC.T.9`
			if (this.commitPointReached() || this.isEmergencyCondition()) {
				newState = State.SLOW;
			}
			break;
		case EMERGENCY: // #State 6 EMERGENCY
			this.setOutput(Speed.STOP, Direction.STOP);
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

	/*
	 * 
	 */
	private boolean commitPointReached() {
		int currPos = mCarLevelPosition.getPosition();
		int desiredPosition = (mDesiredFloor.getFloor() - 1) * 5 * 1000;
		double speed = driveSpeedPayload.speed() * 1000d;
		double stopDist = Math.pow(speed, 2)
				/ (2 * DriveObject.Acceleration * 1000);
		switch (driveSpeedPayload.direction()) {
		case UP:
			return currPos + stopDist + 500 >= desiredPosition;
		case DOWN:
			return currPos - stopDist - 500 <= desiredPosition;
		default:
			return false;
		}

	}

	private void setOutput(Speed speed, Direction direction) {

		drivePayload.set(speed, direction);
		mDrive.set(speed, direction);
		// log(driveSpeedPayload.speed());
		// log(mDriveSpeed.getSpeed());
		mDriveSpeed.set(driveSpeedPayload.speed(),
				driveSpeedPayload.direction());

	}

}
