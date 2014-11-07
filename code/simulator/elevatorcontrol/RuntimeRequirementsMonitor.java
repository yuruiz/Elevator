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
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
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
 * As of Proj10 the summarize() method reports violations of:
 *
 * - R-T10: For each stop at a floor, at least one door reversal shall have occured before the doors are commanded to nudge.
 *
 * @author yuruiz
 */
public class RuntimeRequirementsMonitor extends RuntimeMonitor {
    DoorStateMachine doorState = new DoorStateMachine(new AtFloorArray(canInterface));
    DriveStateMachine driveState = new DriveStateMachine(new AtFloorArray(canInterface));
    boolean hadPendingCall = false;
    boolean hadPendingDoorCall = false;
    boolean[] hadReversal = new boolean[2];
    int totalOpeningCount = 0;
	int wastedOpeningCount = 0;
	int totalStopCount = 0;
	int wastedStopCount = 0;
    int wastedNudgeCount = 0;
    int totalNudgeCount = 0;
	
	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub

	}

	@Override
	protected String[] summarize() {
        String[] arr = new String[3];
        arr[0] = wastedStopCount + " unnecessary stops out of " + totalStopCount + " total.";
        arr[1] = wastedOpeningCount + " unnecessary openings out of " + totalOpeningCount + " total.";
        arr[2] = wastedNudgeCount + " unnecessary nudge out of " + totalNudgeCount + " total";
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

    @Override
    public void receive(ReadableDoorReversalPayload msg) {

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
    private void doorClosed(Hallway hallway, int currentFloor) {
        //System.out.println(hallway.toString() + " Door Closed");
        // Once all doors are closed, check to see if opening was wasted
        if (!hadPendingDoorCall) {
        	warning("Violation of R-T7: Door opened at floor " + currentFloor + " and hallway " + hallway + " where there were no pending calls.");
        	this.wastedOpeningCount += 1;
        }
        hadPendingDoorCall = false;
        totalOpeningCount += 1;
    }
    
    /**
     * Called once when the doors open without a call to the floor
     * @param floor which door the event pertains to
     * @param hallway which door the event pertains to
     */
    private void noCallDoorOpened(int floor, Hallway hallway) {
    	hadPendingDoorCall = false;
        hadReversal[hallway.ordinal()] = false;
    }
    
    /**
     * Called once when the doors open with a call to the floor
     * @param floor which door the event pertains to
     * @param hallway which door the event pertains to
     */
    private void callDoorOpened(int floor, Hallway hallway) {
    	hadPendingDoorCall = true;
        hadReversal[hallway.ordinal()] = false;
    }
    
    /**
     * Called once when the drive is moving
     */
    private void driveMoving(int currentFloor) {
        // Once drive starts moving again, check if stop was wasted
        if (!hadPendingCall) {
        	warning("Violation of R-T6: Drive stopped at floor " + currentFloor + " with no pending calls.");
        	this.wastedStopCount += 1;
        }
        hadPendingCall = false;
        totalStopCount += 1;
    }
    
    /**
     * Called once when the drive is stopped at a floor with no call at that floor
     */
    private void noCallDriveStopped(int floor) {
    	//System.out.println("Drive stopped at floor " + f + " without a call.");
    	hadPendingCall = false;
    }
    
    /**
     * Called once when the drive is stopped at a floor with a call at that floor
     */
    private void callDriveStopped(int floor) {
    	hadPendingCall = true;
    }

    /*
    * Called once when the door is start nudge
    * */
    private void callDoorNudge(Hallway hallway){
        totalNudgeCount++;
        if (!hadReversal[hallway.ordinal()]) {
            warning("Violation of R-T10: Door nudge at " + hallway + " with no reversal triggered.");
            wastedNudgeCount++;
        }

        hadReversal[hallway.ordinal()] = false;
    }

    private void callDoorReversal(Hallway hallway) {
        hadReversal[hallway.ordinal()] = true;
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
        boolean[] isNudging = new boolean[2];
        boolean[] isReversaling =new boolean[2];

        public DoorStateMachine(AtFloorArray atFloors) {
            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
            isNudging[Hallway.FRONT.ordinal()] = false;
            isNudging[Hallway.BACK.ordinal()] = false;
            isReversaling[Hallway.FRONT.ordinal()] = false;
            isReversaling[Hallway.BACK.ordinal()] = false;
            this.atFloorArray = atFloors;
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
            if(doorReversals[h.ordinal()][Side.LEFT.ordinal()].isReversing() || doorReversals[h.ordinal()][Side.RIGHT
                    .ordinal()].isReversing()){
                if(!isReversaling[h.ordinal()]) {
                    callDoorReversal(h);
                    isReversaling[h.ordinal()] = true;
                }
            }else{
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
            		throw new RuntimeException("Doors are open without car being at any floor.");
            	}
            	return;
            }

            if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
            	newState = DoorState.CLOSED;
            } else if (!allDoorsClosed(h) && !hadPendingDoorCall) {
                // Doors opened, check if need to set hadPendingDoorCall
                if (wasCalled(currentFloor, h)) {
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
                    	//System.out.println("Door opened in response to call");
                    	callDoorOpened(currentFloor, h);
                        break;
                    case NO_CALL_OPEN:
                        noCallDoorOpened(currentFloor, h);
                        break;

                }
            }

            //set the newState
            state[h.ordinal()] = newState;
        }

        public void updateNudge(Hallway hallway) {
            if (doorNudge(hallway)) {
                if(!isNudging[hallway.ordinal()]) {
                    callDoorNudge(hallway);
                    isNudging[hallway.ordinal()] = true;
                }
            }else{
                isNudging[hallway.ordinal()] = false;
            }
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
        	return carCalls[f-1][h.ordinal()].isPressed() || 
        			hallCalls[f-1][h.ordinal()][Direction.UP.ordinal()].pressed() || 
        			hallCalls[f-1][h.ordinal()][Direction.DOWN.ordinal()].pressed();
        }

        public boolean doorNudge(Hallway hallway) {
            return doorMotors[hallway.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE && doorMotors[hallway.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE;
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
            
            if (!driveStopped()) {
            	newState = DriveState.MOVING;
            } else if (driveStopped() && !hadPendingCall) {
            	// Drive is stopped, check whether there was a call here or not
            	if (wasCalled(currentFloor)) {
            		newState = DriveState.CALL_STOP;
            	} else {
            		newState = DriveState.NO_CALL_STOP;
            	}
            }
            
            if (newState != this.currentState) {
                switch (newState) {
                    case MOVING:
                        driveMoving(currentFloor);
                        break;
                    case CALL_STOP:
                    	callDriveStopped(currentFloor);
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
        	return carCalls[f-1][h.ordinal()].isPressed() || 
        			hallCalls[f-1][h.ordinal()][Direction.UP.ordinal()].pressed() || 
        			hallCalls[f-1][h.ordinal()][Direction.DOWN.ordinal()].pressed();
        }
    }

}
