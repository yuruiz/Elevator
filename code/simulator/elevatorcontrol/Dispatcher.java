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
import simulator.elevatorcontrol.Utility.*;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightAlarmCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.framework.*;
import simulator.payloads.CanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

import java.util.Arrays;

/**
 * Created by yuruiz on 10/5/14.
 */
public class Dispatcher extends Controller {

    private static enum State {
    	Initial,
        UpUp,
        UpStop,
        UpDown,
        StopUp,
        StopStop,
        StopDown,
        DownUp,
        DownStop,
        DownDown,
        Emergency
    }

    private SimTime period;
    private State currentState;
    private int CurrentFloor;
    private Direction CurrentDirection;
    private Direction DesiredDirection;
    private int Target;
    private int DesiredDwell;
    private int previousFloorSeen;
    // Flags for whether the car can commit to stopping at a particular floor, indexing by floor number
    // Updated at each time instance
    private boolean[] canCommit = new boolean[Elevator.numFloors + 1];
    private final int mmDistBetweenFloors = (int) Elevator.DISTANCE_BETWEEN_FLOORS * 1000;

    /*Network Input*/
    private AtFloorArray mAtFloor;
    private DoorClosedArray mFrontDoorClosed;
    private DoorClosedArray mBackDoorClosed;
    private HallCallArray mHallCallArray;
    private CarCallArray mCarCallArray;
    private CanMailbox.ReadableCanMailbox networkCarWeight;
    private CarWeightAlarmCanPayloadTranslator mCarWeight;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    private CanMailbox.ReadableCanMailbox networkCarLevelPosition;
    private CanMailbox.ReadableCanMailbox networkDriveSpeed;
    private DriveSpeedCanPayloadTranslator mDriveSpeed;

    /*Network Output*/
    private CanMailbox.WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private CanMailbox.WriteableCanMailbox networkFrontDesiredDwell;
    private IntegerCanPayloadTranslator mFrontDesiredDwell;
    private CanMailbox.WriteableCanMailbox networkBackDesiredDwell;
    private IntegerCanPayloadTranslator mBackDesiredDwell;

    


    public Dispatcher(int height, SimTime period, boolean verbose) {
        super("Dispatcher", verbose);

        this.period = period;
        this.Target = 1;
        this.currentState = State.Initial;
        this.DesiredDwell = 1000;
        this.previousFloorSeen = 1;

        mAtFloor = new AtFloorArray(canInterface);
        mFrontDoorClosed = new DoorClosedArray(Hallway.FRONT,canInterface);
        mBackDoorClosed = new DoorClosedArray(Hallway.BACK, canInterface);
        mHallCallArray = new HallCallArray(canInterface);
        mCarCallArray = new CarCallArray(canInterface);
        
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_ALARM_CAN_ID);
        mCarWeight = new CarWeightAlarmCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);

        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
        
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);

        networkFrontDesiredDwell = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT));
        mFrontDesiredDwell = new IntegerCanPayloadTranslator(networkFrontDesiredDwell);
        canInterface.sendTimeTriggered(networkFrontDesiredDwell, period);

        networkBackDesiredDwell = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK));
        mBackDesiredDwell = new IntegerCanPayloadTranslator(networkBackDesiredDwell);
        canInterface.sendTimeTriggered(networkBackDesiredDwell, period);

        mBackDesiredDwell.set(this.DesiredDwell);
        mFrontDesiredDwell.set(this.DesiredDwell);

        mDesiredFloor.set(1, Hallway.BOTH, Direction.STOP);
        timer.start(period);
    }

    @Override
    public void timerExpired(Object callbackData) {
        log("Executing state " + currentState);
        State nextState = currentState;

        CurrentFloor = mAtFloor.getCurrentFloor();
        if (CurrentFloor != -1) {
            this.previousFloorSeen = CurrentFloor;
        }
        
        updateCommitPoints();
        
        /* Avoid duplicate declarations of variables here */
        CallRequest closestCarCallAbove = mCarCallArray.closestCallAbove(previousFloorSeen, this.canCommit);
        CallRequest closestHallCallAbove = mHallCallArray.closestCallAbove(previousFloorSeen, this.canCommit);
        CallRequest closestCarCallBelow = mCarCallArray.closestCallBelow(previousFloorSeen, this.canCommit);
        CallRequest closestHallCallBelow = mHallCallArray.closestCallBelow(previousFloorSeen, this.canCommit);
		
        Hallway desiredHallway = Hallway.NONE;
        CallRequest targetRequest;
        CallRequest carCallBeforeTarget;

        //#transition DPT.12
        if (CurrentFloor == -1 && !(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed())) {
        	currentState = State.Emergency;
        }
        
        switch (currentState) {
        	case Initial:
        		/* Go down to lobby and have no desired floor afterward */
        		CurrentDirection = (CurrentFloor == 1) ? Direction.STOP : Direction.DOWN;
        		DesiredDirection = Direction.STOP;
        		mDesiredFloor.set(Target, DesiredDirection, Hallway.NONE);
        		//#transition DPT.13
        		if (CurrentDirection == Direction.STOP) {
        			nextState = State.StopStop;
        		}
        		break;
        	case StopStop:
        		//DONE
        		CurrentDirection = Direction.STOP;
        		Target = CurrentFloor;
        		DesiredDirection = Direction.STOP;
        		
        		CallRequest curFloorCarCall = mCarCallArray.isCalled(CurrentFloor);
        		desiredHallway = (curFloorCarCall.isValid()) ? curFloorCarCall.hallway : Hallway.NONE;
        		
        		//#transition DPT.6
        		if (closestCarCallAbove.isValid() || closestHallCallAbove.isValid() || 
        				mHallCallArray.isCalled(CurrentFloor, Direction.UP).isValid()) {
        			nextState = State.StopUp;
        		//#transition DPT.7
        		} else if (closestCarCallBelow.isValid() || closestHallCallBelow.isValid() ||
        				mHallCallArray.isCalled(CurrentFloor, Direction.DOWN).isValid()) {
        			nextState = State.StopDown;
        		}
        		mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
        		break;
			case StopDown:
				//DONE
				CurrentDirection = Direction.STOP;
				DesiredDirection = Direction.DOWN;
				targetRequest = computeTarget(closestCarCallBelow, closestHallCallBelow, DesiredDirection);
				if (targetRequest.isValid()) {
					Target = targetRequest.floor;
					desiredHallway = targetRequest.hallway;
				} else {
					if (CurrentFloor != -1) {
						CallRequest curFloorCall = mHallCallArray.isCalled(CurrentFloor, Direction.DOWN);
						if (curFloorCall.isValid()) {
							Target = CurrentFloor;
							desiredHallway = curFloorCall.hallway;
						}
					}
				} // TODO: Handle else case (countdown and go to stop stop)
				
				//#transition DPT.8
				if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() &&
						mDriveSpeed.getSpeed() > DriveObject.LevelingSpeed) {
					nextState = State.DownStop;
				}
					
				mDesiredFloor.set(Target,  DesiredDirection, desiredHallway);
				break;
			case StopUp:
				//DONE
				CurrentDirection = Direction.STOP;
				DesiredDirection = Direction.UP;
				targetRequest = computeTarget(closestCarCallAbove, closestHallCallAbove, DesiredDirection);
				if (targetRequest.isValid()) {
					Target = targetRequest.floor;
					desiredHallway = targetRequest.hallway;
				} else {
					if (CurrentFloor != -1) {
						CallRequest curFloorCall = mHallCallArray.isCalled(CurrentFloor, Direction.UP);
						if (curFloorCall.isValid()) {
							Target = CurrentFloor;
							desiredHallway = curFloorCall.hallway;
						} // TODO: Handle else case (countdown and go to stop stop)
					}
				}
				
				//#transition DPT.2
				if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() &&
						mDriveSpeed.getSpeed() > DriveObject.LevelingSpeed) {
					nextState = State.UpStop;
				}
					
				mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
				break;
        	case UpStop:
        		//DONE
        		CurrentDirection = Direction.UP;
        		DesiredDirection = Direction.STOP;
				targetRequest = computeTarget(closestCarCallAbove, closestHallCallAbove, CurrentDirection);
				if (targetRequest.isValid()) {
					Target = targetRequest.floor;
					desiredHallway = targetRequest.hallway;
				} else {
					//#transition DPT.1
	        		if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() &&
	        				mAtFloor.getCurrentFloor() != -1) {
	        			nextState = State.StopStop;
	        		}
	        		break;
				}
				
        		// Check for calls above Target
        		CallRequest nextCarCallAbove = mCarCallArray.closestCallAbove(Target, canCommit);
        		CallRequest nextHallCallAbove = mHallCallArray.closestCallAbove(Target, canCommit);
        		
        		//#transition DPT.3
        		if ((closestHallCallAbove.isValid() && closestHallCallAbove.direction == Direction.UP) 
        				|| (nextCarCallAbove.isValid() || nextHallCallAbove.isValid())) {
        			nextState = State.UpUp;
        		} 
        		//#transition DPT.4
        		else if (mHallCallArray.isCalled(Target, Direction.DOWN).isValid()) {
        			nextState = State.UpDown;
        		}
        		mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
        		break;
			case DownStop:
				//DONE
				CurrentDirection = Direction.DOWN;
				DesiredDirection = Direction.STOP;
				targetRequest = computeTarget(closestCarCallBelow, closestHallCallBelow, CurrentDirection);
				if (targetRequest.isValid()) {
					Target = targetRequest.floor;
					desiredHallway = targetRequest.hallway;
				} else {
					//#transition DPT.1
	        		if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() &&
	        				mAtFloor.getCurrentFloor() != -1) {
	        			nextState = State.StopStop;
	        		}
	        		break;
				}
				
        		// Check for calls below Target
        		CallRequest nextCarCallBelow = mCarCallArray.closestCallBelow(Target, canCommit);
        		CallRequest nextHallCallBelow = mHallCallArray.closestCallBelow(Target, canCommit);
        		
        		//#transition DPT.9
        		if ((closestHallCallBelow.isValid() && closestHallCallBelow.direction == Direction.DOWN) 
        				|| (nextCarCallBelow.isValid() || nextHallCallBelow.isValid())) {
        			nextState = State.DownDown;
        		} 
        		//#transition DPT.10
        		else if (mHallCallArray.isCalled(Target, Direction.UP).isValid()){
        			nextState = State.DownUp;
        		}
        		mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
        		break;	
			case UpUp:
				//DONE
				CurrentDirection = Direction.UP;
				DesiredDirection = Direction.UP;
				
				CallRequest upUpHallCall = mHallCallArray.closestCallAboveInDirection(previousFloorSeen, Direction.UP, canCommit);
				targetRequest = computeTarget(upUpHallCall, closestCarCallAbove, Direction.UP);
				if (targetRequest.isValid()) {
					Target = targetRequest.floor;
					desiredHallway = targetRequest.hallway;
				} else {
					//#transition DPT.1
	        		if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() &&
	        				mAtFloor.getCurrentFloor() != -1) {
	        			nextState = State.StopUp;
	        		}
	        		break;
				}

				//#transition DPT.1
				if (!mFrontDoorClosed.getBothClosed() && mAtFloor.getCurrentFloor() != -1) {
					nextState = State.StopUp;
				}
				mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
				break;
			case DownDown:
				//DONE
				CurrentDirection = Direction.DOWN;
				DesiredDirection = Direction.DOWN;
				CallRequest downDownHallCall = mHallCallArray.closestCallBelowInDirection(previousFloorSeen, Direction.DOWN, canCommit);
				
				targetRequest = computeTarget(downDownHallCall, closestCarCallBelow, Direction.DOWN);
				if (targetRequest.isValid()) {
					Target = targetRequest.floor;
					desiredHallway = targetRequest.hallway;
				} else {
					//#transition DPT.1
	        		if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() &&
	        				mAtFloor.getCurrentFloor() != -1) {
	        			nextState = State.StopDown;
	        		}
	        		break;
				}
				
				//#transition DPT.1
				if (!mFrontDoorClosed.getBothClosed() && mAtFloor.getCurrentFloor() != -1) {
					nextState = State.StopDown;
				}
				mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
				break;
			case UpDown:
				CurrentDirection = Direction.UP;
				DesiredDirection = Direction.DOWN;

                // Find the minimum hall call below the target
                CallRequest minDownHallCallAboveTarget = mHallCallArray.maxGoingDown(Target-1, canCommit);
				carCallBeforeTarget = mCarCallArray.lowestCallBetween(previousFloorSeen, Target, canCommit);
				if (minDownHallCallAboveTarget.isValid()) {
                    Target = minDownHallCallAboveTarget.floor;
                    desiredHallway = minDownHallCallAboveTarget.hallway;
				}
				
        		//#transition DPT.5
				if (carCallBeforeTarget.isValid()) {
                    // There is a CarCall before existing target, switch to DownDown
                    nextState = State.UpUp;
	    		//#transition DPT.1
	        	} else if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) ||
                                mAtFloor.getCurrentFloor() != -1) {
					nextState = State.StopDown;
				}
				
				mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
				break;
			case DownUp:
				CurrentDirection = Direction.DOWN;
				DesiredDirection = Direction.UP;

                // Find the minimum hall call below the target
                CallRequest minUpHallCallBelowTarget = mHallCallArray.minGoingUp(Target+1, canCommit);
				carCallBeforeTarget = mCarCallArray.highestCallBetween(previousFloorSeen, Target, canCommit);
				if (minUpHallCallBelowTarget.isValid()) {
                    Target = minUpHallCallBelowTarget.floor;
                    desiredHallway = minUpHallCallBelowTarget.hallway;
				}
				
        		//#transition DPT.11
				if (carCallBeforeTarget.isValid()) {
                    // There is a CarCall before existing target, switch to DownDown
                    nextState = State.DownDown;
	    		//#transition DPT.1
	        	} else if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) ||
                                mAtFloor.getCurrentFloor() != -1) {
					nextState = State.StopUp;
				}
				
				mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
				break;
			case Emergency:
                Target = 1;
                DesiredDirection = Direction.STOP;
                desiredHallway = Hallway.NONE;

				mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
				break;
			default:
				log("State is invalid in dispatcher.");
				break;
        }
        
        if (currentState != nextState) {
        	log("Transition from " + currentState + " --> " + nextState);
        }
        System.out.println("Desired Floor: " + mDesiredFloor.getFloor() + " " + mDesiredFloor.getDirection() + "  " +  mDesiredFloor.getHallway());
        this.currentState = nextState;
        timer.start(period);
    }
    
    private CallRequest computeTarget(CallRequest closestCarCall,
			CallRequest closestHallCall, Direction dir) {
    	
    		int floor;
    		Hallway hallway;
			if (!(closestCarCall.isValid() || closestHallCall.isValid())) {
				return new CallRequest();
			} else if (!closestCarCall.isValid()) {
				// There is only a hall call below the current floor
				floor = closestHallCall.floor;
				hallway = closestHallCall.hallway;
			} else if (!closestHallCall.isValid()) {
				// There is only a car call below the current floor
				floor = closestCarCall.floor;
				hallway = closestCarCall.hallway;
			} else {
				// There is both a hall and car call below the current floor, compute the closest one
				switch (dir) {
					case UP:
						if (closestCarCall.floor < closestHallCall.floor) {
							floor = closestHallCall.floor;
							hallway = closestHallCall.hallway;
						} else if (closestCarCall.floor == closestHallCall.floor) {
                            floor = closestCarCall.floor;
                            hallway = CallRequest.union(closestCarCall.hallway, closestHallCall.hallway);
                        } else {
                            floor = closestCarCall.floor;
                            hallway = closestCarCall.hallway;
                        }
						break;
					case DOWN:
						if (closestCarCall.floor > closestHallCall.floor) {
							floor = closestHallCall.floor;
							hallway = closestHallCall.hallway;
						} else if (closestCarCall.floor == closestHallCall.floor) {
                            floor = closestCarCall.floor;
                            hallway = CallRequest.union(closestCarCall.hallway, closestHallCall.hallway);
                        } else {
							floor = closestCarCall.floor;
							hallway = closestCarCall.hallway;
						}
						break;
					default:
						return new CallRequest();
					}
			}
			return new CallRequest(floor, Direction.STOP, hallway);
	}

	void updateCommitPoints() {
    	/* Update this.commitPoints array here */
    	double currSpeed = mDriveSpeed.getSpeed();
    	Direction currDir = mDriveSpeed.getDirection();
    	int currPos = mCarLevelPosition.getPosition();
    	
    	//|x| = vi^2/(2*a)
    	double stoppingDistance = (currSpeed * currSpeed)/(2 * DriveObject.Acceleration);
    	double stoppingPoint;
    	int nearestFloor;
    	
    	switch (currDir) {
	    	case STOP:
	    		/* If stopped at a floor, it is possible to reach any floor */
	    		Arrays.fill(this.canCommit, true);
	    		break;
	    	case UP:
	    		/* Moving up: All floors above stopping point can be reached */
	    		stoppingPoint = currPos + stoppingDistance;
	    		nearestFloor = (int) Math.ceil(stoppingPoint/this.mmDistBetweenFloors) + 1;
	    		Arrays.fill(this.canCommit, false);
	    		
	    		for (int i = nearestFloor; i <= Elevator.numFloors; i++) {
	    			this.canCommit[i] = true;
	    		}
	    		
	    		break;
	    	case DOWN:
	    		/* Moving down: All floors below stopping point can be reached */
	    		stoppingPoint = currPos - stoppingDistance;
	    		nearestFloor = (int) Math.floor(stoppingPoint/this.mmDistBetweenFloors) + 1;

	    		for (int i = nearestFloor; i >= 1; i--) {
	    			this.canCommit[i] = true;
	    		}
	    		break;
    	}
    }
}
