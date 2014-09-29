package simulator.elevatormodules;

import simulator.elevatorcontrol.MessageDictionary;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;

/**
 * Created by yuruiz on 9/29/14.
 */
public class DoorMotorCanPayloadTranslator extends VarIntegerCanTranslator {
    public DoorMotorCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway, Side side) {
        super(payload, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId
                (hallway, side), "DoorMotorCommand" + ReplicationComputer.computeReplicationId(hallway, side), 1);
    }

    public DoorMotorCanPayloadTranslator(CanMailbox.WriteableCanMailbox payload, Hallway hallway, Side side) {
        super(payload, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId
                (hallway, side), "DoorMotorCommand" + ReplicationComputer.computeReplicationId(hallway, side), 1);
    }

    public void setValue(DoorCommand command) {
        switch (command) {
            case OPEN:
                super.setValue(0);
                break;
            case CLOSE:
                super.setValue(1);
                break;
            case STOP:
                super.setValue(2);
                break;
            case NUDGE:
                super.setValue(3);
                break;
            default:
                throw new RuntimeException("State " + command + " was not recognized.");
        }
    }


    public DoorCommand getCommand() {
        int value = super.getValue();

        switch (value) {
            case 0:
                return DoorCommand.OPEN;
            case 1:
                return DoorCommand.CLOSE;
            case 2:
                return DoorCommand.STOP;
            case 3:
                return DoorCommand.NUDGE;
            default:
                throw new RuntimeException("State " + value + " was not recognized.");
        }

    }
}

