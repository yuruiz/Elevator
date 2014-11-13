package simulator.elevatorcontrol;

import simulator.payloads.CanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

import java.util.BitSet;

/**
 * Created by yuruiz on 11/13/14.
 */

public class EmergencyBreakCanPayloadTranslator extends CanPayloadTranslator {


    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public EmergencyBreakCanPayloadTranslator(CanMailbox.WriteableCanMailbox payload) {
        super(payload, 1);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public EmergencyBreakCanPayloadTranslator(CanMailbox.ReadableCanMailbox payload) {
        super(payload, 1);
    }

    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }


    public void setValue(boolean value) {
        BitSet b = getMessagePayload();
        b.set(0, value);
        setMessagePayload(b, getByteSize());
    }

    public boolean getValue() {
        return getMessagePayload().get(0);
    }

    @Override
    public String payloadToString() {
        return Boolean.toString(getValue());
    }
}

