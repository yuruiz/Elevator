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
        Initial, UpUp, UpStop, UpDown, StopUp, StopStop, StopDown, DownUp, DownStop, DownDown, Emergency
    }

    private SimTime period;
    private State currentState;
    private int CurrentFloor;
    private Direction CurrentDirection;
    private Direction DesiredDirection;
    private int Target;
    private int CountDown;
    private int DesiredDwell;
    private int previousFloorSeen;
    private long cycleCount = 0;
    // Flags for whether the car can commit to stopping at a particular floor,
    // indexing by floor number
    // Updated at each time instance
    private boolean[] canCommit = new boolean[Elevator.numFloors + 1];
    private final int mmDistBetweenFloors = (int) Elevator.DISTANCE_BETWEEN_FLOORS * 1000;

    /* Network Input */
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

    /* Network Output */
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
        this.DesiredDwell = 300;
        this.previousFloorSeen = 1;

        mAtFloor = new AtFloorArray(canInterface);
        mFrontDoorClosed = new DoorClosedArray(Hallway.FRONT, canInterface);
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
//        int practicalFloor;
        // CurrentFloor = mAtFloor.getCurrentFloor();
        CurrentFloor = getApproxCurrentFloor();
        if (CurrentFloor != -1) {
            this.previousFloorSeen = CurrentFloor;
        }

        Hallway desiredHallway = Hallway.NONE;
        updateCommitPoints();

		/* Avoid duplicate declarations of variables here */
        CallRequest closestCarCallAbove = mCarCallArray.closestCallAbove(previousFloorSeen, this.canCommit);
        CallRequest closestHallCallAbove = mHallCallArray.closestCallAbove(previousFloorSeen, this.canCommit);
        CallRequest closestCarCallBelow = mCarCallArray.closestCallBelow(previousFloorSeen, this.canCommit);
        CallRequest closestHallCallBelow = mHallCallArray.closestCallBelow(previousFloorSeen, this.canCommit);

        CallRequest closesetCarCallAboveEuqal = null;
        CallRequest closesetCarCallBelowEuqal = null;
        CallRequest targetRequest;
        CallRequest carCallBeforeTarget;

        // #transition DPT.12
        if (CurrentFloor == -1 && !(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed())) {
            nextState = State.Emergency;
            currentState = nextState;
            System.out.println("Now in Emergency State!!");
        }

//        System.out.println("Current floor is " + CurrentFloor);
//        System.out.println("Executing state " + currentState);
        switch (currentState) {
            case Initial:
            /* Go down to lobby and have no desired floor afterward */
                CurrentDirection = (CurrentFloor == 1) ? Direction.STOP : Direction.DOWN;
                DesiredDirection = Direction.STOP;
                mDesiredFloor.set(Target, DesiredDirection, Hallway.NONE);
                // #transition DPT.13
                if (CurrentDirection == Direction.STOP) {
                    nextState = State.StopStop;
                }
                break;
            case StopStop:
                // DONE
                CurrentDirection = Direction.STOP;
                Target = CurrentFloor;
                DesiredDirection = Direction.STOP;
                CountDown = DesiredDwell * 2;

                CallRequest curFloorCarCall = mCarCallArray.isCalled(CurrentFloor);
                desiredHallway = (curFloorCarCall.isValid()) ? curFloorCarCall.hallway : Hallway.NONE;

                // #transition DPT.6
                if (closestCarCallAbove.isValid() || closestHallCallAbove.isValid() || mHallCallArray.isCalled(CurrentFloor, Direction.UP).isValid()) {
                    if (desiredHallway != Hallway.NONE) {
                        if (!mBackDoorClosed.getBothClosed() || !mFrontDoorClosed.getBothClosed()) {
                            nextState = State.StopUp;
                        }
                    }else{
                        nextState = State.StopUp;
                    }

                    // #transition DPT.7
                } else if (closestCarCallBelow.isValid() || closestHallCallBelow.isValid() || mHallCallArray.isCalled(CurrentFloor, Direction.DOWN).isValid()) {
                    if (desiredHallway != Hallway.NONE) {
                        if (!mBackDoorClosed.getBothClosed() || !mFrontDoorClosed.getBothClosed()) {
                            nextState = State.StopDown;
                        }
                    }else{
                        nextState = State.StopDown;
                    }
                }
                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case StopDown:
                // DONE
                CurrentDirection = Direction.STOP;
                DesiredDirection = Direction.DOWN;
                CountDown -= 1;
                closesetCarCallBelowEuqal = mCarCallArray.closestCallBelowEqual(previousFloorSeen, this.canCommit);
//                CallRequest closesetHallCallBelowEuqal = mHallCallArray.closestCallBelowEqual(previousFloorSeen, this.canCommit);
                targetRequest = computeTarget(closesetCarCallBelowEuqal, closestHallCallBelow, DesiredDirection);

//                System.out.println("Hall call " + closestHallCallBelow.floor);
//                System.out.println("Target " + targetRequest.floor);
                if (!mBackDoorClosed.getBothClosed() || !mFrontDoorClosed.getBothClosed()) {
                    CallRequest CurrentFloorHallCall = mHallCallArray.isCalled(CurrentFloor);
                    if (CurrentFloorHallCall.isValid() && CurrentFloorHallCall.direction == Direction.DOWN) {
                        Target = CurrentFloorHallCall.floor;
                        desiredHallway = CurrentFloorHallCall.hallway;
                    } else if (targetRequest.isValid()) {
                        Target = targetRequest.floor;
                        desiredHallway = targetRequest.hallway;
                    }
                } else {
                    CallRequest newTarget = computeTarget(closestCarCallBelow, closestHallCallBelow, DesiredDirection);
                    if (newTarget.isValid()) {
                        Target = newTarget.floor;
                        desiredHallway = newTarget.hallway;
                    }
                }

                // #transition DPT.8
                if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() && mDriveSpeed.getSpeed() > DriveObject.LevelingSpeed) {
                    nextState = State.DownStop;
                    // #transition DPT.14
                } else if (CountDown <= 0 && !(closestCarCallBelow.isValid() || closestHallCallBelow.isValid()) &&
                        mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) {
                    nextState = State.StopStop;
                }

                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case StopUp:
                // DONE
                CurrentDirection = Direction.STOP;
                DesiredDirection = Direction.UP;
                CountDown -= 1;

                closesetCarCallAboveEuqal = mCarCallArray.closestCallAboveEqual(previousFloorSeen, this.canCommit);
//                CallRequest closesetHallCallAboveEuqal = mHallCallArray.closestCallAboveEqual(previousFloorSeen, this.canCommit);
                targetRequest = computeTarget(closesetCarCallAboveEuqal, closestHallCallAbove, DesiredDirection);

                if (!mBackDoorClosed.getBothClosed() || !mFrontDoorClosed.getBothClosed()) {
                    CallRequest CurrentFloorHallCall = mHallCallArray.isCalled(CurrentFloor);
                    if (CurrentFloorHallCall.isValid() && CurrentFloorHallCall.direction == Direction.UP) {
                        Target = CurrentFloorHallCall.floor;
                        desiredHallway = CurrentFloorHallCall.hallway;
                    } else if (targetRequest.isValid()) {
                        Target = targetRequest.floor;
                        desiredHallway = targetRequest.hallway;
                    }
                } else {
                    CallRequest newTarget = computeTarget(closestCarCallAbove, closestHallCallAbove, DesiredDirection);
                    if (newTarget.isValid()) {
                        Target = newTarget.floor;
                        desiredHallway = newTarget.hallway;
                    }
                }

//                System.out.println("Closest car call is " + closestCarCallAbove.floor);
//                System.out.println("Closest hall call is " + closestHallCallAbove.floor + closestHallCallAbove.direction);
                // #transition DPT.2
                if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed() && mDriveSpeed.getSpeed() > DriveObject.LevelingSpeed) {
                    nextState = State.UpStop;
                    // #transition DPT.14
                } else if (CountDown <= 0 && !(closestHallCallAbove.isValid() || closestCarCallAbove.isValid()) &&
                        mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) {
                    nextState = State.StopStop;
                }

                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case UpStop:
                // DONE
                CurrentDirection = Direction.UP;
                DesiredDirection = Direction.STOP;
                CountDown = DesiredDwell * 2;

//                practicalFloor = getPracticalFloor(previousFloorSeen, CurrentDirection);

                closesetCarCallAboveEuqal = mCarCallArray.closestCallAboveEqual(previousFloorSeen, this.canCommit);
//                targetRequest = computeTarget(closesetCarCallAboveEuqal, closestHallCallAbove, CurrentDirection);
                if(closesetCarCallAboveEuqal.isValid() && mCarCallArray.isCalled(Target).isValid()) {
                    if (closesetCarCallAboveEuqal.floor <= Target) {
                        Target = closesetCarCallAboveEuqal.floor;
                        desiredHallway = closesetCarCallAboveEuqal.hallway;
                    }else{
                        desiredHallway = mCarCallArray.isCalled(Target).hallway;
                    }
                } else if (mCarCallArray.isCalled(Target).isValid()) {
                    desiredHallway = mCarCallArray.isCalled(Target).hallway;
                } else if (mHallCallArray.isCalled(Target, Direction.UP).isValid()) {
                    nextState = State.UpUp;
                    break;
                } else if (closesetCarCallAboveEuqal.isValid()) {
                    Target = closesetCarCallAboveEuqal.floor;
                    desiredHallway = closesetCarCallAboveEuqal.hallway;
                } else if (mHallCallArray.isCalled(Target, Direction.DOWN).isValid()) {
                    nextState = State.UpDown;
                    break;
                } else {
                    // #transition DPT.1
                    if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) && CurrentFloor != -1) {
                        nextState = State.StopStop;
                        break;
                    }
                }

//                System.out.println("Closeset Car Call Above Equal " + closesetCarCallAboveEuqal.floor);
//                System.out.println("Previous Seen Floor is " + previousFloorSeen + this.canCommit[previousFloorSeen]);

                // Check for calls above Target
                CallRequest nextCarCallAbove = mCarCallArray.closestCallAbove(Target, canCommit);
                CallRequest nextHallCallAbove = mHallCallArray.closestCallAboveInDirection(Target, Direction.UP, canCommit);

                // #transition DPT.3

                if ((closestHallCallAbove.isValid() && closestHallCallAbove.direction == Direction.UP) || (nextCarCallAbove.isValid() || nextHallCallAbove.isValid())) {
                    nextState = State.UpUp;
                } else if (mHallCallArray.isCalled(Target, Direction.DOWN).isValid() && !closestCarCallAbove.isValid() && !mCarCallArray.isCalled(Target).isValid()) {
                    nextState = State.UpDown;
                }

//                System.out.println("Closeset Car Call Above " + closestCarCallAbove.floor + closestCarCallAbove.isValid());
//                System.out.println("Target is " + Target);
                // #transition DPT.4
                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case DownStop:
                // DONE
                CurrentDirection = Direction.DOWN;
                DesiredDirection = Direction.STOP;
                CountDown = DesiredDwell * 2;

//                practicalFloor = getPracticalFloor(previousFloorSeen, CurrentDirection);

                closesetCarCallBelowEuqal = mCarCallArray.closestCallBelowEqual(previousFloorSeen, this.canCommit);

//                targetRequest = computeTarget(closesetCarCallBelowEuqal, closestHallCallBelow, CurrentDirection);


                if(closesetCarCallBelowEuqal.isValid() && mCarCallArray.isCalled(Target).isValid()) {
                    if (closesetCarCallBelowEuqal.floor >= Target) {
                        Target = closesetCarCallBelowEuqal.floor;
                        desiredHallway = closesetCarCallBelowEuqal.hallway;
                    }else{
                        desiredHallway = mCarCallArray.isCalled(Target).hallway;
                    }
                } else if (mCarCallArray.isCalled(Target).isValid()) {
                    desiredHallway = mCarCallArray.isCalled(Target).hallway;
                } else if (mHallCallArray.isCalled(Target, Direction.DOWN).isValid()) {
                    nextState = State.DownDown;
                    break;
                } else if (closesetCarCallBelowEuqal.isValid()) {
                    Target = closesetCarCallBelowEuqal.floor;
                    desiredHallway = closesetCarCallBelowEuqal.hallway;
                } else if(mHallCallArray.isCalled(Target, Direction.UP).isValid()){
                    nextState = State.DownUp;
                    break;
                } else {
                    // #transition DPT.1
                    if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) && CurrentFloor != -1) {
                        nextState = State.StopStop;
                    }
                    break;
                }

                // Check for calls below Target
                CallRequest nextCarCallBelow = mCarCallArray.closestCallBelow(Target, canCommit);
                CallRequest nextHallCallBelow = mHallCallArray.closestCallBelowInDirection(Target, Direction.DOWN, canCommit);

                // #transition DPT.9

                if ((closestHallCallBelow.isValid() && closestHallCallBelow.direction == Direction.DOWN) || (nextCarCallBelow.isValid() || nextHallCallBelow.isValid())) {
                    nextState = State.DownDown;
                } else if (mHallCallArray.isCalled(Target, Direction.UP).isValid() && !closestCarCallBelow.isValid() && !mCarCallArray.isCalled(Target).isValid()) {
                    nextState = State.DownUp;
                }
                // #transition DPT.10
                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case UpUp:
                // DONE
                CurrentDirection = Direction.UP;
                DesiredDirection = Direction.UP;
                CountDown--;


//                practicalFloor = getPracticalFloor(previousFloorSeen, CurrentDirection);

                CallRequest upUpHallCall = mHallCallArray.closestCallAboveInDirection(previousFloorSeen, Direction.UP, canCommit);
                closesetCarCallAboveEuqal = mCarCallArray.closestCallAboveEqual(previousFloorSeen, this.canCommit);
                targetRequest = computeTarget(closesetCarCallAboveEuqal, upUpHallCall, Direction.UP);
                if (targetRequest.isValid() && targetRequest.floor <= Target) {
                    Target = targetRequest.floor;
                    desiredHallway = targetRequest.hallway;
                }

//                System.out.println("Closest car call is " + closesetCarCallAboveEuqal.floor);
//                System.out.println("Closest hall call is " + upUpHallCall.floor + upUpHallCall.direction);
//                System.out.println("Practical Floor is "+ practicalFloor + canCommit[practicalFloor]);


                // #transition DPT.1
                if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) && CurrentFloor != -1) {
                    CountDown = DesiredDwell * 2;
                    nextState = State.StopUp;
                    break;
                } else if(CountDown <= 0 && CurrentFloor != -1 && mDriveSpeed.getSpeed() == 0 && desiredHallway == Hallway.NONE){
                    nextState = State.StopStop;
                }
                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case DownDown:
                // DONE
                CurrentDirection = Direction.DOWN;
                DesiredDirection = Direction.DOWN;
                CountDown--;

//                System.out.println("Previous Seen " + previousFloorSeen);

//                practicalFloor = getPracticalFloor(previousFloorSeen, CurrentDirection);

                CallRequest downDownHallCall = mHallCallArray.closestCallBelowInDirection(previousFloorSeen, Direction.DOWN,
                        canCommit);

                closesetCarCallBelowEuqal = mCarCallArray.closestCallBelowEqual(previousFloorSeen, this.canCommit);
                targetRequest = computeTarget(closesetCarCallBelowEuqal, downDownHallCall, Direction.DOWN);

                if (targetRequest.isValid() && targetRequest.floor >= Target) {
                    Target = targetRequest.floor;
                    desiredHallway = targetRequest.hallway;
                }


                // #transition DPT.1
                if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) && CurrentFloor != -1) {
                    CountDown = DesiredDwell * 2;
                    nextState = State.StopDown;
                    break;
                }else if(CountDown <= 0 && CurrentFloor != -1 && mDriveSpeed.getSpeed() == 0 && desiredHallway == Hallway.NONE){
                    nextState = State.StopStop;
                }
                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case UpDown:
                CurrentDirection = Direction.UP;
                DesiredDirection = Direction.DOWN;
                CountDown = DesiredDwell * 2;

                // Find the minimum hall call below the target
                CallRequest minDownHallCallAboveTarget = mHallCallArray.maxGoingDown(Target - 1, canCommit);
                // Get the next up hall call above the current floor
//                practicalFloor = getPracticalFloor(previousFloorSeen, CurrentDirection);
                CallRequest upHallCallAboveCurFloor = mHallCallArray.closestCallAboveInDirection(previousFloorSeen, Direction.UP,
                        canCommit);
                carCallBeforeTarget = mCarCallArray.lowestCallBetween(previousFloorSeen, Target, canCommit);
                if (minDownHallCallAboveTarget.isValid() && mDriveSpeed.getSpeed() >= DriveObject.SlowSpeed) {
                    Target = minDownHallCallAboveTarget.floor;
                    CallRequest curTarget = mCarCallArray.isCalled(Target);
                    if (curTarget.isValid()) {
                        desiredHallway = CallRequest.union(minDownHallCallAboveTarget.hallway, curTarget.hallway);
                    } else {
                        desiredHallway = minDownHallCallAboveTarget.hallway;
                    }
                } else if (mHallCallArray.isCalled(Target, Direction.DOWN).isValid()) {
                    CallRequest CurHalltarget = mHallCallArray.isCalled(Target, Direction.DOWN);
                    CallRequest curTarget = mCarCallArray.isCalled(Target);
                    if (curTarget.isValid()) {
                        desiredHallway = CallRequest.union(CurHalltarget.hallway, curTarget.hallway);
                    } else {
                        desiredHallway = CurHalltarget.hallway;
                    }
                }

                // #transition DPT.5
                if (carCallBeforeTarget.isValid() || upHallCallAboveCurFloor.isValid()) {
                    // Car call before target or Hall Call in the up direction => Go
                    // to UpUp
                    nextState = State.UpUp;
                    // #transition DPT.17
                } else if (mCarCallArray.closestCallAbove(Target, canCommit).isValid()) {
                    nextState = State.UpStop;
                    // #transition DPT.1
                } else if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) && CurrentFloor != -1) {
                    nextState = State.StopDown;
                }

                mDesiredFloor.set(Target, DesiredDirection, desiredHallway);
                break;
            case DownUp:
                CurrentDirection = Direction.DOWN;
                DesiredDirection = Direction.UP;
                CountDown = DesiredDwell * 2;

                // Find the minimum hall call below the target
                CallRequest minUpHallCallBelowTarget = mHallCallArray.minGoingUp(Target + 1, canCommit);
                // Get the next down hall call below the current floor
//                practicalFloor = getPracticalFloor(previousFloorSeen, CurrentDirection);
                CallRequest downHallCallBelowCurFloor = mHallCallArray.closestCallBelowInDirection(previousFloorSeen, Direction.DOWN,
                        canCommit);
                carCallBeforeTarget = mCarCallArray.highestCallBetween(previousFloorSeen, Target, canCommit);
                if (minUpHallCallBelowTarget.isValid() && mDriveSpeed.getSpeed() >= DriveObject.SlowSpeed) {
                    Target = minUpHallCallBelowTarget.floor;
                    CallRequest curTarget = mCarCallArray.isCalled(Target);
                    if (curTarget.isValid()) {
                        desiredHallway = CallRequest.union(minUpHallCallBelowTarget.hallway, curTarget.hallway);
                    } else {
                        desiredHallway = minUpHallCallBelowTarget.hallway;
                    }
                } else if (mHallCallArray.isCalled(Target, Direction.UP).isValid()) {
                    CallRequest CurHalltarget = mHallCallArray.isCalled(Target, Direction.UP);
                    CallRequest curTarget = mCarCallArray.isCalled(Target);
                    if (curTarget.isValid()) {
                        desiredHallway = CallRequest.union(CurHalltarget.hallway, curTarget.hallway);
                    } else {
                        desiredHallway = CurHalltarget.hallway;
                    }
                }

                // #transition DPT.11
                if (carCallBeforeTarget.isValid() || downHallCallBelowCurFloor.isValid()) {
                    nextState = State.DownDown;
                    // #transition DPT.16
                } else if (mCarCallArray.closestCallBelow(Target, canCommit).isValid()) {
                    nextState = State.DownStop;
                    // #transition DPT.1
                } else if (!(mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) && CurrentFloor != -1) {
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

        cycleCount++;
        if (currentState != nextState) {
            log("Transition from " + currentState + " --> " + nextState);
//            System.out.println("Transition from " + currentState + " --> " + nextState);
        }

        // System.out.println("Desired Floor: " + mDesiredFloor.getFloor() + " "
        // + mDesiredFloor.getDirection() + "  " + mDesiredFloor.getHallway());
        this.currentState = nextState;
        timer.start(period);
    }

    private CallRequest computeTarget(CallRequest closestCarCall, CallRequest closestHallCall, Direction dir) {

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
            // There is both a hall and car call below the current floor,
            // compute the closest one
            if (closestHallCall.direction != dir) {
                floor = closestCarCall.floor;
                hallway = closestCarCall.hallway;
            } else {
                switch (dir) {
                    case UP:
                        if (closestCarCall.floor < closestHallCall.floor) {
                            floor = closestCarCall.floor;
                            hallway = closestCarCall.hallway;
                        } else if (closestCarCall.floor == closestHallCall.floor) {
                            floor = closestCarCall.floor;
                            hallway = CallRequest.union(closestCarCall.hallway, closestHallCall.hallway);
                        } else {
                            floor = closestHallCall.floor;
                            hallway = closestHallCall.hallway;
                        }

                        break;
                    case DOWN:
                        if (closestCarCall.floor > closestHallCall.floor) {
                            floor = closestCarCall.floor;
                            hallway = closestCarCall.hallway;
                        } else if (closestCarCall.floor == closestHallCall.floor) {
                            floor = closestCarCall.floor;
                            hallway = CallRequest.union(closestCarCall.hallway, closestHallCall.hallway);
                        } else {
                            floor = closestHallCall.floor;
                            hallway = closestHallCall.hallway;
                        }

                        break;
                    default:
                        return new CallRequest();
                }
            }
        }

        return new CallRequest(floor, Direction.STOP, hallway);

    }

    /*
     * Helper to return approximation to current floor based on position. -1 if
     * not close to a floor
     */
    private int getApproxCurrentFloor() {
        int threshold_down = 200; // in mm.
        int threshold_up = 4800;
        int currPos = mCarLevelPosition.getPosition();
        int error = currPos % this.mmDistBetweenFloors;
        if (error < threshold_down || error > threshold_up) {
            return ((currPos + threshold_down) / this.mmDistBetweenFloors) + 1;
        } else {
            return -1;
        }
    }

    private int getPracticalFloor(int curFloor, Direction direction) {
        int currPos = mCarLevelPosition.getPosition();

        if (currPos < 0) {
            return 1;
        } else if (currPos > 7 * this.mmDistBetweenFloors) {
            return 8;
        }

        if (mDriveSpeed.getSpeed() < DriveObject.LevelingSpeed) {
            return curFloor;
        }

        switch (direction) {
            case UP:
                if (currPos >= (curFloor - 1) * this.mmDistBetweenFloors) {
                    return curFloor + 1;
                }
                break;
            case DOWN:
                if (currPos <= (curFloor - 1) * this.mmDistBetweenFloors) {
                    return curFloor - 1;
                }
                break;
            default:
                break;
        }

        return curFloor;

    }

    void updateCommitPoints() {
        /* Update this.commitPoints array here */
        double currSpeed = mDriveSpeed.getSpeed() * 1000d;
        Direction currDir = mDriveSpeed.getDirection();
        int currPos = mCarLevelPosition.getPosition();
        double stoppingDistance = 0;

        // |x| = vi^2/(2*a)

        stoppingDistance = (currSpeed * currSpeed) / (2 * DriveObject.Acceleration * 1000);

        if (mDriveSpeed.getSpeed() <= DriveObject.LevelingSpeed) {
            Arrays.fill(this.canCommit, true);
            return;
        }


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
                nearestFloor = (int) Math.ceil(stoppingPoint / this.mmDistBetweenFloors) + 1;
                Arrays.fill(this.canCommit, false);

                for (int i = nearestFloor; i <= Elevator.numFloors; i++) {
                    this.canCommit[i] = true;
                }

                break;
            case DOWN:
            /* Moving down: All floors below stopping point can be reached */
                stoppingPoint = currPos - stoppingDistance;
                nearestFloor = (int) Math.floor(stoppingPoint / this.mmDistBetweenFloors) + 1;
                Arrays.fill(this.canCommit, false);


                for (int i = nearestFloor; i >= 1; i--) {
                    this.canCommit[i] = true;
                }
                break;
        }
    }
}

