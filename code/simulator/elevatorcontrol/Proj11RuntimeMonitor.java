package simulator.elevatorcontrol;

import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;

/**
 * High level requirements runtime monitor.
 * 
 * As of Proj8, the summarize() method reports violations of:
 * 	- RT-6: The Car shall only stop at Floors for which there are pending calls.
 *  - RT-7: The Car shall only open Doors at Hallways for which there are pending calls.
 * @author vijay
 *
 */
public class Proj11RuntimeMonitor extends RuntimeMonitor {
    DoorStateMachine doorState = new DoorStateMachine(new AtFloorArray(canInterface));
    DriveStateMachine driveState = new DriveStateMachine(new AtFloorArray(canInterface));
    boolean wasWastedOpening = false;
    boolean wasWastedStop = false;
    int totalOpeningCount = 0;
	int wastedOpeningCount = 0;
	int totalStopCount = 0;
	int wastedStopCount = 0;
	
	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub

	}

	@Override
	protected String[] summarize() {
        String[] arr = new String[2];
        arr[0] = wastedStopCount + " unnecessary stops out of " + totalStopCount + " total.";
        arr[1] = wastedOpeningCount + " unnecessary openings out of " + totalOpeningCount + " total.";
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
    }
    
    @Override
    public void receive(ReadableDriveSpeedPayload msg) {
        driveState.receive(msg);
    }

    /**************************************************************************
     * high level event methods
     *
     * these are called by the logic in the message receiving methods and the
     * state machines
     **************************************************************************/

    /**
     * Called once when the doors close completely
     * @param hallway which door the event pertains to
     */
    private void doorClosed(Hallway hallway) {
        //System.out.println(hallway.toString() + " Door Closed");
        // Once all doors are closed, check to see if opening was wasted
        if (wasWastedOpening) {
        	this.wastedOpeningCount += 1;
        }
        wasWastedOpening = false;
        totalOpeningCount += 1;
    }
    
    /**
     * Called once when the drive is stopped at a floor with no call at that floor
     */
    private void noCallDriveStopped(int floor) {
    	//System.out.println("Drive stopped at floor " + f + " without a call.");
    	wasWastedStop = true;
    }
    
    /**
     * Called once when the drive is moving
     */
    private void driveMoving() {
        // Once all doors are closed, check to see if opening was wasted
        if (wasWastedStop) {
        	this.wastedStopCount += 1;
        }
        wasWastedStop = false;
        totalStopCount += 1;
    }
    
    /**
     * Called once when the doors open without a call to the floor
     * @param floor which door the event pertains to
     * @param hallway which door the event pertains to
     */
    private void noCallDoorOpened(int floor, Hallway hallway) {
    	wasWastedOpening = true;
    }
	
    private static enum DoorState {
        CLOSED,
        CALL_OPEN,
        NO_CALL_OPEN
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

        public DoorStateMachine(AtFloorArray atFloors) {
            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
            this.atFloorArray = atFloors;
        }

        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorMotorPayload msg) {
            updateState(msg.getHallway());
        }

        private void updateState(Hallway h) {
            DoorState previousState = state[h.ordinal()];

            DoorState newState = previousState;
            int currentFloor = atFloorArray.getCurrentFloor();
            if (currentFloor == MessageDictionary.NONE) {
            	// Not at any floor, doors must be closed.
            	// No other action required
            	if (!allDoorsClosed(h)) {
            		throw new RuntimeException("Doors are open without car being at any floor.");
            	}
            	return;
            }
            
            if (!allDoorsClosed(h) && !wasCalled(currentFloor, h)) {
            	// Doors opening without a call at this floor/hallway
                newState = DoorState.NO_CALL_OPEN;
            } else if (!allDoorsClosed(h) && wasCalled(currentFloor, h)) {
            	// Doors opening with a call at this floor/hallway
                newState = DoorState.CALL_OPEN;
            } else if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
            	// Doors are closed
                newState = DoorState.CLOSED;
            }
            
            if (newState != previousState) {
                switch (newState) {
                    case CLOSED:
                        doorClosed(h);
                        break;
                    case CALL_OPEN:
                    	//System.out.println("Door opened in response to call");
                        break;
                    case NO_CALL_OPEN:
                        noCallDoorOpened(currentFloor, h);
                        break;
                }
            }

            //set the newState
            state[h.ordinal()] = newState;
        }

        //door utility methods
        public boolean allDoorsClosed(Hallway h) {
            return (doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    && doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed());
        }

        public boolean allDoorMotorsStopped(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP && doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
        }

        // Returns whether there a car or hall call to this floor/hallway combination
        public boolean wasCalled(int f, Hallway h){
        	return carCalls[f][h.ordinal()].isPressed() || 
        			hallCalls[f][h.ordinal()][Direction.UP.ordinal()].pressed() || 
        			hallCalls[f][h.ordinal()][Direction.DOWN.ordinal()].pressed();
        }
    }

    private static enum DriveState {
        MOVING,
        CALL_STOP,
        NO_CALL_STOP
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

        public DriveStateMachine(AtFloorArray atFloors) {
            this.currentState = DriveState.CALL_STOP;
            this.atFloorArray = atFloors;
        }

        public void receive(ReadableDrivePayload msg) {
            updateState();
        }

        public void receive(ReadableDriveSpeedPayload msg) {
            updateState();
        }

        private void updateState() {
            DriveState newState = this.currentState;
            
            int currentFloor = atFloorArray.getCurrentFloor();
            if (currentFloor == MessageDictionary.NONE) {
            	return;
            }
            
            if (driveStopped() && !wasCalled(currentFloor)) {
            	// Drive stopped without a call at this floor
                newState = DriveState.NO_CALL_STOP;
            } else if (driveStopped() && wasCalled(currentFloor)) {
            	// Drive stopped with a call at this floor
                newState = DriveState.CALL_STOP;
            } else if (!driveStopped()) {
            	// Drive is moving
                newState = DriveState.MOVING;
            }
            
            if (newState != this.currentState) {
                switch (newState) {
                    case MOVING:
                        driveMoving();
                        break;
                    case CALL_STOP:
                    	//System.out.println("Drive stopped in response to call");
                        break;
                    case NO_CALL_STOP:
                        noCallDriveStopped(currentFloor);
                        break;
                }
            }

            //set the newState
            this.currentState = newState;
        }

        //drive utility methods
        public boolean driveStopped() {
        	return Speed.isStopOrLevel(driveCommandedSpeed.speed());
        }
        
        // Checks whether calls were made from either hallway
        public boolean wasCalled(int f) {
        	return (wasCalled(f, Hallway.BACK) || wasCalled(f, Hallway.FRONT));
        }

        // Returns whether there a car or hall call to this floor/hallway combination
        public boolean wasCalled(int f, Hallway h){
        	return carCalls[f][h.ordinal()].isPressed() || 
        			hallCalls[f][h.ordinal()][Direction.UP.ordinal()].pressed() || 
        			hallCalls[f][h.ordinal()][Direction.DOWN.ordinal()].pressed();
        }
    }

}