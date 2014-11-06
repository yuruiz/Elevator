/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
 */

package simulator.elevatorcontrol;

import java.nio.ByteBuffer;
import java.util.Arrays;
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
	public void set(double speed, Direction dir) {
		setSpeed(speed);
		setDirection(dir);
	}

	public void setSpeed(double speed) {
		BitSet b = getMessagePayload();
		float f = (float) speed;
		byte[] bytes = ByteBuffer.allocate(4).putFloat(f).array();
		b.clear(0, 32);
		b.or(BitSet.valueOf(bytes));
		setMessagePayload(b, getByteSize());
	}

	public double getSpeed() {
		BitSet b = getMessagePayload();
		BitSet temp = b.get(0, 32);
		byte[] bytes = new byte[4];
		byte[] nums = temp.toByteArray();
		int i = 0;

		// Fill the buffer with ending 0s
		while (i < 4) {
			if (i < nums.length) {
				bytes[i] = nums[i];
			} else {
				bytes[i] = 0;
			}
			i++;
		}

		float f = ByteBuffer.wrap(bytes).getFloat();
		double val = (double) f;
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