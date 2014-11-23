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
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Elevator;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;

public class CarPositionControl extends Controller {

    private int CurrentFloor;
    private SimTime period;
    private State state;
    private double mmDistBetweenFloors = Elevator.DISTANCE_BETWEEN_FLOORS * 1000;

    // input
    private AtFloorArray mAtFloor;
    private ReadableCanMailbox networkCarLevelPosition;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    private ReadableCanMailbox networkDriveSpeed;
    private DriveSpeedCanPayloadTranslator mDriveSpeed;

    // output
    private WriteableCarPositionIndicatorPayload localCarPositionIndicator;

    private enum State {
        ARRIVE, MOVING
    }

    public CarPositionControl(SimTime period, boolean verbose) {
        super("CarPositionControl", verbose);
        this.period = period;
        state = State.ARRIVE;
        CurrentFloor = 1; // the first floor

		/* Input */
        mAtFloor = new AtFloorArray(canInterface);
        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);

        canInterface.registerTimeTriggered(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkCarLevelPosition);

		/* Output */

        localCarPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
        physicalInterface.sendTimeTriggered(localCarPositionIndicator, period);

        timer.start(period);
    }

    @Override
    public void timerExpired(Object callbackData) {

//        State nextState = state;
        int currPos = mCarLevelPosition.getPosition();
        log("State " + state);

        localCarPositionIndicator.set(CurrentFloor);
        if (currPos < 0) {
            CurrentFloor = 1;
        } else if (currPos % this.mmDistBetweenFloors >= 2500) {
            CurrentFloor = (int) Math.ceil(currPos / this.mmDistBetweenFloors) + 1;
        } else {
            CurrentFloor = (int) Math.floor(currPos / this.mmDistBetweenFloors) + 1;
        }

//        if (state != nextState) {
//            log("Transition: " + state + " -> " + nextState);
//        }
//        state = nextState;
        timer.start(period);
    }
}
