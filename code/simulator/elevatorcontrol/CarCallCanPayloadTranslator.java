package simulator.elevatorcontrol;

import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Created by yuruiz on 10/5/14.
 */
public class CarCallCanPayloadTranslator extends CanPayloadTranslator {

    public CarCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
        super(payload, 16, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
    }


    public CarCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
        super(payload, 16, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
    }
    @Override
    public String payloadToString() {
        return null;
    }
}
