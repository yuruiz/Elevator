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
	private static final double MAX_DELAY = 500d;

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
	private Direction currentDirection = Direction.STOP;
	private int ClosedCount = 0;

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
		/*
		 * If the elevator is ready to set new direction
		 */

		int desiredPosition = (mDesiredFloor.getFloor() - 1) * 5000;
		/*
		 * If desired position is higher
		 */
		if (mCarLevelPosition.getPosition() < desiredPosition) {
			/*
			 * If currently moving down but speed is slower than slow speed
			 */
			if (driveSpeedPayload.direction() == Direction.DOWN) {
				if (driveSpeedPayload.speed() <= DriveObject.SlowSpeed) {
					desiredDirection = Direction.STOP;
				}
			} else {
				desiredDirection = Direction.UP;
			}
		}
		// if desired position is lower
		else if (mCarLevelPosition.getPosition() > desiredPosition) {
			/*
			 * if moving in the opposite direction but speed is slow
			 */
			if (driveSpeedPayload.direction() == Direction.UP) {
				if (driveSpeedPayload.speed() <= DriveObject.SlowSpeed) {
					desiredDirection = Direction.STOP;
				}
			} else {
				desiredDirection = Direction.DOWN;
			}
		}

		/*
		 * Prevent direction directly changes from UP to DOWN or DOWN to UP
		 */
		if ((currentDirection == Direction.UP && desiredDirection == Direction.DOWN)
				|| (currentDirection == Direction.DOWN && desiredDirection == Direction.UP)) {
			desiredDirection = Direction.STOP;
		}

		currentDirection = desiredDirection;

		/*
		 * check door all closed
		 */
		boolean allClosed = doorClosedFront.getBothClosed()
				&& doorClosedBack.getBothClosed();

		State newState = currentState;
		switch (currentState) {
		case STOP: // #State 1 STOP
			this.setOutput(Speed.STOP, Direction.STOP);

			if (allClosed) {
				ClosedCount++;
			} else {
				ClosedCount = 0;
			}

			// #transition 'DC.T.1'
			if (!mLevelUp.getValue()
					&& driveSpeedPayload.direction() != Direction.DOWN
					&& driveSpeedPayload.speed() == 0) {
				newState = State.LEVEL_UP;
				break;
			}

			// #transition 'DC.T.3'
			if (!mLevelDown.getValue()
					&& driveSpeedPayload.direction() != Direction.UP
					&& driveSpeedPayload.speed() == 0) {
				newState = State.LEVEL_DOWN;
				break;
			}
			// log(allClosed + " " + currentFloor + mDesiredFloor.getFloor());
			// #transition 'DC.T.5'
			if (allClosed && mCarWeight.getValue() < Elevator.MaxCarCapacity
					&& atFloor.getCurrentFloor() != mDesiredFloor.getFloor()
					&& ClosedCount > 35 && driveSpeedPayload.speed() == 0) {
				log("stop to slow");
				ClosedCount = 0;
				newState = State.SLOW;
			}
			break;
		case SLOW: // #State 4 SLOW
			ClosedCount = 0;
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
				if (!mLevelUp.getValue()
						&& driveSpeedPayload.direction() == Direction.UP) {
					newState = State.LEVEL_UP;
				}
				// #transition `DC.T.7`
				else {
					if (driveSpeedPayload.direction() == Direction.DOWN) {
						log("slow to level down");
						newState = State.LEVEL_DOWN;
					}
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
			ClosedCount = 0;
			this.setOutput(Speed.LEVEL, Direction.UP);
			// #transition 'DC.T.2'
			if (mLevelUp.getValue()) {
				// System.out.println("level up to stop");
				newState = State.STOP;
			}
			break;
		case LEVEL_DOWN: // #State 3 LEVEL DOWN
			ClosedCount = 0;
			this.setOutput(Speed.LEVEL, Direction.DOWN);
			// #transition `DC.T.4`
			if (mLevelDown.getValue()) {
				log("level down to stop");
				newState = State.STOP;
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

		if (wrongDirection(desiredPosition)) {
			return true;
		}

		if (currPos < desiredPosition) {

			return (currPos + stopDist + speed * 0.11 + 100) >= desiredPosition;
		} else {
			if (currPos > desiredPosition) {
				return (currPos - stopDist - speed * 0.11 - 100) <= desiredPosition;
			} else {
				return true;
			}
		}

	}

	private boolean wrongDirection(int desiredPosition) {
		if (mCarLevelPosition.getPosition() < desiredPosition
				&& driveSpeedPayload.direction() == Direction.DOWN) {
			return true;
		}

		if (mCarLevelPosition.getPosition() > desiredPosition
				&& driveSpeedPayload.direction() == Direction.UP) {
			return true;
		}

		return false;

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
