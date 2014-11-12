/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
*/
package simulator.elevatorcontrol;

import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

import java.util.BitSet;

/**
 * Created by yuruiz on 9/30/14.
 */
public class DoorMotorCanPayloadTranslator extends CanPayloadTranslator{
    public DoorMotorCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway, Side side) {
        super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId
                (hallway, side));
    }

    public DoorMotorCanPayloadTranslator(WriteableCanMailbox payload, Hallway hallway, Side side) {
        super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId
                (hallway, side));
    }

    public void setCommand(DoorCommand command) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, command.ordinal(), 0, getByteSize() * 8);
        setMessagePayload(b, getByteSize());
    }

    public DoorCommand getCommand() {
        int val = getIntFromBitset(getMessagePayload(), 0, getByteSize() * 8);
        for (DoorCommand d : DoorCommand.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override
    public String payloadToString() {
        return "DoorCommand = " + getCommand();
    }
}
