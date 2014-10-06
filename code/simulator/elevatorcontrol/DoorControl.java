package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarWeightAlarmCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorOpenedCanPayloadTranslator;
import simulator.framework.*;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

/**
 * Created by yuruiz on 9/28/14.
 */
public class DoorControl extends Controller {
    public static enum State {
        Opening,
        Opened,
        Closed,
        Nudge
    }

    private static final int height = 8;
    private static final int MaxCarCapacity = 10000;

    private SimTime period;
    private Hallway hallway;
    private Side side;
    private int dwell;
    private int countdown;

    /*Output Physical Interface*/
    private WriteableDoorMotorPayload localDoorMotor;

    /*Input network Interface*/
//    private ReadableCanMailbox networkAtFloor[];
//    private AtFloorCanPayloadTranslator mAtFloor[];
    private ReadableCanMailbox networkDesiredDwell;
    private IntegerCanPayloadTranslator mDesiredDwell;
    private ReadableCanMailbox networkDoorClosed;
    private DoorClosedCanPayloadTranslator mDoorClosed;
    private ReadableCanMailbox networkDoorOpened;
    private DoorOpenedCanPayloadTranslator mDoorOpened;
    private ReadableCanMailbox networkCarWeight;
    private CarWeightAlarmCanPayloadTranslator mCarWeight;
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private Utility.AtFloorArray mAtFloor;

    /*Input Physical Interface*/
    private ReadableDriveSpeedPayload localDriveSpeed;

    /*Output Network Interface*/
    private WriteableCanMailbox networkDoorMotor;
    private DoorMotorCanPayloadTranslator mDoorMotor;

    State currentState;

    public DoorControl(SimTime period, Hallway hallway, Side side, boolean verbose) {
        super("DoorControl" + ReplicationComputer.makeReplicationString(hallway, side), verbose);

        this.period = period;
        this.hallway = hallway;
        this.side = side;
        this.dwell = 0;
        this.countdown = this.dwell;

//        networkAtFloor = new ReadableCanMailbox[height];

//        mAtFloor = new AtFloorCanPayloadTranslator[height];
//        for (int i = 1; i <= height; i++) {
//            networkAtFloor[i - 1] = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(i, hallway));
//            mAtFloor[i-1] = new AtFloorCanPayloadTranslator(networkAtFloor[i - 1], i, hallway);
//            canInterface.registerTimeTriggered(networkAtFloor[i - 1]);
//        }
        mAtFloor = new Utility.AtFloorArray(canInterface);

        networkDesiredDwell = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway));
        mDesiredDwell = new IntegerCanPayloadTranslator(networkDesiredDwell);
        canInterface.registerTimeTriggered(networkDesiredDwell);

        networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
        canInterface.registerTimeTriggered(networkDoorClosed);

        networkDoorOpened = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
        mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
        canInterface.registerTimeTriggered(networkDoorOpened);

        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_ALARM_CAN_ID);
        mCarWeight = new CarWeightAlarmCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);

        networkDesiredFloor = CanMailbox.getReadableCanMailbox((MessageDictionary.DESIRED_FLOOR_CAN_ID));
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);

        networkDoorMotor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
        mDoorMotor = new DoorMotorCanPayloadTranslator(networkDoorMotor, hallway, side);
        canInterface.sendTimeTriggered(networkDoorMotor, period);


        localDriveSpeed = DriveSpeedPayload.getReadablePayload();
        physicalInterface.registerTimeTriggered(localDriveSpeed);

        localDoorMotor = DoorMotorPayload.getWriteablePayload(hallway, side);
        physicalInterface.sendTimeTriggered(localDoorMotor, period);


        localDoorMotor.set(DoorCommand.STOP);
        mDoorMotor.setCommand(DoorCommand.STOP);
//        while (!mDoorClosed.getValue()){}
        this.currentState = State.Closed;

        timer.start(period);
    }

    @Override
    public void timerExpired(Object callbackData) {
        log("Executing state " + currentState);

        switch (currentState) {
            case Opening:    /*State 1 Opening*/
                localDoorMotor.set(DoorCommand.OPEN);
                mDoorMotor.setCommand(DoorCommand.OPEN);
                dwell = mDesiredDwell.getValue();
                countdown = dwell;
                //#transition T.1
                if (mDoorOpened.getValue()) {
                    currentState = State.Opened;
                }
                break;
            case Opened:    /*State 2 Opened*/
                localDoorMotor.set(DoorCommand.STOP);
                mDoorMotor.setCommand(DoorCommand.STOP);
                dwell = mDesiredDwell.getValue();
                countdown--;

                //#transition T.2
                if (countdown < 0 && !mCarWeight.getValue()) {
                    currentState = State.Nudge;
                }
                break;
            case Nudge:    /*State 4 Nudge*/
                localDoorMotor.set(DoorCommand.NUDGE);
                mDoorMotor.setCommand(DoorCommand.STOP);
                dwell = mDesiredDwell.getValue();

                //#transition T.3
                if (mDoorClosed.getValue()) {
                    currentState = State.Closed;
                }
                //#transition T.5
                else if (mCarWeight.getValue()) {
                    currentState = State.Opening;
                }
                break;
            case Closed:  /*State 3 Closed*/
//                log("Start Door Closed State");
                localDoorMotor.set(DoorCommand.STOP);
                mDoorMotor.setCommand(DoorCommand.STOP);
                dwell = mDesiredDwell.getValue();

                //#transition T.4
                if (mCarWeight.getValue() || (atFloor() == mDesiredFloor.getFloor() && mDesiredFloor.getHallway() ==
                        hallway && localDriveSpeed.speed() == 0) || localDriveSpeed.direction() == Direction.STOP) {
                    currentState = State.Opening;
                }
                break;
            default:
                throw new RuntimeException("State " + currentState + " was not recognized.");
        }

        timer.start(period);

    }

    private int atFloor() {
        for (int i = 0; i < height; i++) {
            if(this.mAtFloor.isAtFloor(i+1, hallway)){
                return i;
            }
        }
        return 0;
    }
}
