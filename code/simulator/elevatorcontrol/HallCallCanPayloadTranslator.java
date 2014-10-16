/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
*/

package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

;
;

/**
 * Created by yuruiz on 10/5/14.
 */
public class HallCallCanPayloadTranslator extends CanPayloadTranslator{

    public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, 16, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
    }

    public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, 16, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
    }

    @Override
    public String payloadToString() {
        return null;
    }
}
