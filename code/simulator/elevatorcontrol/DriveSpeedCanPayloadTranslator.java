/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
*/

package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.elevatorcontrol.MessageDictionary;
import simulator.framework.Direction;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {

	public DriveSpeedCanPayloadTranslator(ReadableCanMailbox payload) {
		super(payload, 8, MessageDictionary.DRIVE_SPEED_CAN_ID);
	}

	public DriveSpeedCanPayloadTranslator(WriteableCanMailbox payload) {
		super(payload, 8, MessageDictionary.DRIVE_SPEED_CAN_ID);
	}

	/**
	 * This method is required for setting values by reflection in the
	 * MessageInjector. The order of parameters in .mf files should match the
	 * signature of this method. All translators must have a set() method with
	 * the signature that contains all the parameter values.
	 *
	 * @param speed
	 * @param dir
	 */
	public void set(int speed, Direction dir) {
		setSpeed(speed);
		setDirection(dir);
	}

	public void setSpeed(int speed) {
		BitSet b = getMessagePayload();
		addIntToBitset(b, speed, 0, 32);
		setMessagePayload(b, getByteSize());
	}

	public int getSpeed() {
		int val = getIntFromBitset(getMessagePayload(), 0, 32);
		if (val >= 0) {
			return val;
		}
		throw new RuntimeException("Unrecognized Speed Value " + val);
	}

	public void setDirection(Direction dir) {
		BitSet b = getMessagePayload();
		addIntToBitset(b, dir.ordinal(), 32, 32);
		setMessagePayload(b, getByteSize());
	}

	public Direction getDirection() {
		int val = getIntFromBitset(getMessagePayload(), 32, 32);
		for (Direction d : Direction.values()) {
			if (d.ordinal() == val) {
				return d;
			}
		}
		throw new RuntimeException("Unrecognized Direction Value " + val);
	}

	@Override
	public String payloadToString() {
		return "DriveSpeed:  speed=" + getSpeed() + " direction="
				+ getDirection();
	}

}