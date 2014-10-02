package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class HallCallCanPayloadTranslator extends CanPayloadTranslator {

	public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
		super(payload, 8, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
	}
	
	public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
		super(payload, 8, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
	}
	
    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param value
     */
    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }
    
    public void setValue(boolean value) {       
        BitSet b = new BitSet(32);
        b.set(31, value);
        setMessagePayload(b, getByteSize());
        System.out.println(payloadToString());
    }
    
    public boolean getValue() {
        return getMessagePayload().get(31);
    }
    
    @Override
    public String payloadToString() {
        return Boolean.toString(getValue());
    }
}
