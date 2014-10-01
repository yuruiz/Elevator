package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.*;
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
    private ReadableCanMailbox networkAtFloor[];
    private AtFloorCanPayloadTranslator mAtFloor[];
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
        this.dwell = 10000;
        this.countdown = this.dwell;

        physicalInterface.sendTimeTriggered(localDoorMotor, period);

        networkAtFloor = new ReadableCanMailbox[height];

        mAtFloor = new AtFloorCanPayloadTranslator[height];
        for (int i = 0; i < height; i++) {
            networkAtFloor[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(i, hallway));
            mAtFloor[i] = new AtFloorCanPayloadTranslator(networkAtFloor[i], i, hallway);
            canInterface.registerTimeTriggered(networkAtFloor[i]);
        }

        networkDesiredDwell = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway));
        mDesiredDwell = new IntegerCanPayloadTranslator(networkDesiredDwell);
        canInterface.registerTimeTriggered(networkDesiredDwell);

        networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
        canInterface.registerTimeTriggered(networkDoorClosed);

        networkDoorOpened = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
        mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
        canInterface.registerTimeTriggered(networkDoorOpened);

        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
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


        localDoorMotor.set(DoorCommand.CLOSE);
        while (!mDoorClosed.getValue()){}
        this.currentState = State.Closed;



        timer.start(period);
    }

    @Override
    public void timerExpired(Object callbackData) {
        log("Executing state" + currentState);

        switch (currentState) {
            case Opening:
                localDoorMotor.set(DoorCommand.OPEN);
                mDoorMotor.setCommand(DoorCommand.OPEN);
                dwell = mDesiredDwell.getValue();
                countdown = dwell;

                if (mDoorOpened.getValue()) {
                    currentState = State.Opened;
                }
                break;
            case Opened:
                localDoorMotor.set(DoorCommand.STOP);
                mDoorMotor.setCommand(DoorCommand.STOP);
                dwell = mDesiredDwell.getValue();
                countdown--;
                if (countdown < 0 && !mCarWeight.getValue()) {
                    currentState = State.Nudge;
                }
                break;
            case Nudge:
                localDoorMotor.set(DoorCommand.NUDGE);
                mDoorMotor.setCommand(DoorCommand.STOP);
                dwell = mDesiredDwell.getValue();
                if (mDoorClosed.getValue()) {
                    currentState = State.Closed;
                } else if (mCarWeight.getValue()) {
                    currentState = State.Opening;
                }
                break;
            case Closed:
                localDoorMotor.set(DoorCommand.STOP);
                mDoorMotor.setCommand(DoorCommand.STOP);
                dwell = mDesiredDwell.getValue();
                if (mCarWeight.getValue() || atFloor() == mDesiredFloor.getFloor() ||
                        localDriveSpeed.direction() == Direction.STOP) {
                    currentState = State.Opening;
                }
                break;
            default:
                throw new RuntimeException("State " + currentState + " was not recognized.");
        }

    }

    private int atFloor() {
        for (int i = 0; i < height; i++) {
            if (mAtFloor[i].getValue()) {
                return i+1;
            }
        }
        return 0;
    }
}
