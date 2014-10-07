package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarWeightAlarmCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

/**
 * Created by yuruiz on 10/5/14.
 */
public class Dispatcher extends Controller {

    private static enum State{
        Initial,
        Error,
        Target,
        Open,
        Close
    }




    private SimTime period;
    private State CurrentState;
    private int Target;
    private int prevTarget;
    private int DiesiredDewell;
    private boolean Target_set;

    /*Network Input*/
    private Utility.AtFloorArray mAtFloor;
    private Utility.DoorClosedArray mFrontDoorClosed;
    private Utility.DoorClosedArray mBackDoorClosed;
    private Utility.HallCallArray mHallCallArray;
    private Utility.CarCallArray mCarCallArray;
    private CanMailbox.ReadableCanMailbox networkCarWeight;
    private CarWeightAlarmCanPayloadTranslator mCarWeight;

    /*Network Output*/
    private CanMailbox.WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private CanMailbox.WriteableCanMailbox networkFrontDesiredDwell;
    private IntegerCanPayloadTranslator mFrontDesiredDwell;
    private CanMailbox.WriteableCanMailbox networkBackDesiredDwell;
    private IntegerCanPayloadTranslator mBackDesiredDwell;



    public Dispatcher(SimTime period, boolean verbose) {
        super("Dispatcher", verbose);

        this.period = period;
        this.Target = 1;
        this.prevTarget = 1;
        this.CurrentState = State.Initial;
        this.DiesiredDewell = 5;
        this.Target_set = false;

        mAtFloor = new Utility.AtFloorArray(canInterface);
        mFrontDoorClosed = new Utility.DoorClosedArray(Hallway.FRONT,canInterface);
        mBackDoorClosed = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
        mHallCallArray = new Utility.HallCallArray(canInterface);
        mCarCallArray = new Utility.CarCallArray(canInterface);

        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_ALARM_CAN_ID);
        mCarWeight = new CarWeightAlarmCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);

        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);

        networkFrontDesiredDwell = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT));
        mFrontDesiredDwell = new IntegerCanPayloadTranslator(networkFrontDesiredDwell);
        canInterface.sendTimeTriggered(networkFrontDesiredDwell, period);

        networkBackDesiredDwell = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK));
        mBackDesiredDwell = new IntegerCanPayloadTranslator(networkBackDesiredDwell);
        canInterface.sendTimeTriggered(networkBackDesiredDwell, period);

        mBackDesiredDwell.set(this.DiesiredDewell);
        mFrontDesiredDwell.set(this.DiesiredDewell);
        timer.start(period);

    }

    @Override
    public void timerExpired(Object callbackData) {
        log("Executing state " + CurrentState);

        boolean FrontClosed;
        boolean BackClosed;
        boolean atCurrentTarget;
        switch (CurrentState) {
            case Initial:
                this.Target = 1;
                mDesiredFloor.set(Target, Hallway.BOTH, Direction.STOP);

                FrontClosed = mFrontDoorClosed.getBothClosed();
                BackClosed = mBackDoorClosed.getBothClosed();
                if (((FrontClosed == false && mAtFloor.isAtFloor(this.Target, Hallway.FRONT)) || (BackClosed == false && mAtFloor.isAtFloor(this.Target, Hallway.BACK))) && this.Target == 1 &&
                        mDesiredFloor.getHallway() == Hallway.BOTH) {
                    CurrentState = State.Target;
                }
                break;
            case Target:
                if (Target_set == true) {
                    atCurrentTarget = (mAtFloor.isAtFloor(prevTarget, Hallway.FRONT) || mAtFloor.isAtFloor(prevTarget, Hallway.BACK));
                }
                else{
                    atCurrentTarget = (mAtFloor.isAtFloor(Target, Hallway.FRONT) || mAtFloor.isAtFloor(Target, Hallway.BACK));
                    prevTarget = Target;
                    Target = Target % 8 + 1;
                    setTarget(Target, mAtFloor.isAtFloor(Target, Hallway.FRONT), mAtFloor.isAtFloor(Target, Hallway.BACK));
                    Target_set = true;
                }

                if (atCurrentTarget == true && mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) {
                    Target_set = false;
                    CurrentState = State.Close;
                } else if ((!mFrontDoorClosed.getBothClosed() || !mBackDoorClosed.getBothClosed()) && atCurrentTarget == false) {
                    Target_set = false;
                    CurrentState = State.Error;
                }
                break;
            case Open:
                atCurrentTarget = (mAtFloor.isAtFloor(Target, Hallway.FRONT) || mAtFloor.isAtFloor(Target, Hallway.BACK));
                setTarget(Target, mAtFloor.isAtFloor(Target, Hallway.FRONT), mAtFloor.isAtFloor(Target, Hallway.BACK));

                if (Target == mAtFloor.getCurrentFloor() && ((mAtFloor.isAtFloor(Target, Hallway.FRONT) && !mFrontDoorClosed.getBothClosed()) || (mAtFloor.isAtFloor(Target, Hallway.BACK) && !mBackDoorClosed.getBothClosed()))) {

                    CurrentState = State.Target;
                } else if ((!mFrontDoorClosed.getBothClosed() || !mBackDoorClosed.getBothClosed()) && atCurrentTarget == false) {

                    CurrentState = State.Error;
                }
                break;
            case Close:
                atCurrentTarget = (mAtFloor.isAtFloor(Target, Hallway.FRONT) || mAtFloor.isAtFloor(Target, Hallway.BACK));
                mDesiredFloor.set(this.Target, Direction.STOP, Hallway.NONE);

                if (atCurrentTarget == true && (mFrontDoorClosed.getBothClosed() || mBackDoorClosed.getBothClosed() && mDesiredFloor.getHallway() == Hallway.NONE)) {
                    CurrentState = State.Open;
                } else if ((!mFrontDoorClosed.getBothClosed() || !mBackDoorClosed.getBothClosed()) && atCurrentTarget == false) {

                    CurrentState = State.Error;
                }
                break;
            case Error:
                this.Target = 1;
                mDesiredFloor.set(this.Target, Direction.STOP, Hallway.NONE);

                FrontClosed = mFrontDoorClosed.getBothClosed();
                BackClosed = mBackDoorClosed.getBothClosed();
                if (((FrontClosed == false && mAtFloor.isAtFloor(this.Target, Hallway.FRONT)) || (BackClosed == false && mAtFloor.isAtFloor(this.Target, Hallway.BACK))) && this.Target == 1 &&
                        mDesiredFloor.getHallway() == Hallway.NONE) {
                    CurrentState = State.Target;
                }
                break;
            default:
                throw new RuntimeException("State " + CurrentState + " was not recognized.");
        }

        timer.start(period);
    }

    private void setTarget(int target, boolean Front, boolean Back) {
        if (Front == true && Back == true) {
            mDesiredFloor.set(Target, Hallway.BOTH, Direction.STOP);
        }
        else if (Front == true) {
            mDesiredFloor.set(Target, Hallway.FRONT, Direction.STOP);
        }
        else if (Back == true) {
            mDesiredFloor.set(Target, Hallway.BACK, Direction.STOP);
        }
        else {
            mDesiredFloor.set(Target, Hallway.NONE, Direction.STOP);
        }


    }
}
