/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.*;
import simulator.payloads.CANNetwork;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

import java.util.HashMap;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot.  It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they constitute
 * a communication channel between controllers.
 *
 * @author justinr2
 */
public class Utility {

    public static class DoorClosedArray {

        HashMap<Integer, DoorClosedCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
        public final Hallway hallway;

        public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
            this.hallway = hallway;
            for (Side s : Side.values()) {
                int index = ReplicationComputer.computeReplicationId(hallway, s);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + index);
                DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(m, hallway, s);
                conn.registerTimeTriggered(m);
                translatorArray.put(index, t);
            }
        }

        public boolean getBothClosed() {
            return translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue() &&
                    translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.RIGHT)).getValue();
        }
    }

    public static class AtFloorArray {

        public HashMap<Integer, AtFloorCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, AtFloorCanPayloadTranslator>();
        public final int numFloors = Elevator.numFloors;

        public AtFloorArray(CANNetwork.CanConnection conn) {
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                    AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(m, floor, h);
                    conn.registerTimeTriggered(m);
                    networkAtFloorsTranslators.put(index, t);
                }
            }
        }
        
        public boolean isAtFloor(int floor, Hallway hallway) {
            return networkAtFloorsTranslators.get(ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
        }

        public int getCurrentFloor() {
            int retval = MessageDictionary.NONE;
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    AtFloorCanPayloadTranslator t = networkAtFloorsTranslators.get(index);
                    if (t.getValue()) {
                        if (retval == MessageDictionary.NONE) {
                            //this is the first true atFloor
                            retval = floor;
                        } else if (retval != floor) {
                            //found a second floor that is different from the first one
                            throw new RuntimeException("AtFloor is true for more than one floor at " + Harness.getTime());
                        }
                    }
                }
            }
            return retval;
        }
    }

    public static class HallCallArray{
        private CANNetwork.CanConnection conn;
        public HashMap<Integer, BooleanCanPayloadTranslator> translatorArray = new HashMap<Integer, BooleanCanPayloadTranslator>();
        public HallCallArray(CANNetwork.CanConnection conn) {
            this.conn = conn;
            for (int i = 1; i <= 8; i++) {
                switch (i) {
                    case 1:
                        CreateTranslator(i, Hallway.FRONT, Direction.UP);
                        CreateTranslator(i, Hallway.FRONT, Direction.DOWN);
                        CreateTranslator(i, Hallway.BACK, Direction.UP);
                        CreateTranslator(i, Hallway.BACK, Direction.DOWN);
                        break;
                    case 2:
                        CreateTranslator(i, Hallway.BACK, Direction.UP);
                        CreateTranslator(i, Hallway.BACK, Direction.DOWN);
                        break;
                    case 7:
                        CreateTranslator(i, Hallway.FRONT, Direction.UP);
                        CreateTranslator(i, Hallway.FRONT, Direction.DOWN);
                        CreateTranslator(i, Hallway.BACK, Direction.UP);
                        CreateTranslator(i, Hallway.BACK, Direction.DOWN);
                        break;
                    default:
                        CreateTranslator(i, Hallway.FRONT, Direction.UP);
                        CreateTranslator(i, Hallway.FRONT, Direction.DOWN);
                        break;
                }

            }
        }
        
        public CallRequest closestCallAboveInDirection(int curFloor, Direction direction, boolean[] canCommit) {
        	// Called by current floor up, return true
        	CallRequest c;
        	// Called by one of higher floors
        	for (int i = curFloor + 1; i <= Elevator.numFloors; i++) {
        		if ((c = isCalled(i, direction)).isValid() && canCommit[i]) {
        			return c;
        		}
        	}
        	return new CallRequest();
        }
        
        public CallRequest closestCallBelowInDirection(int curFloor, Direction direction, boolean[] canCommit) {
        	CallRequest c;
        	// Called by one of lower floors
        	for (int i = curFloor - 1; i >= 1; i--) {
        		if ((c = isCalled(i, direction)).isValid() && canCommit[i]) {
        			return c;
        		}
        	}
        	return new CallRequest();
        }
    	/* Return true if called at a floor above the current floor or up by curFloor */
        public CallRequest closestCallAbove(int curFloor, boolean[] canCommit) {
        	// Called by current floor up, return true
        	CallRequest c;
        	// Called by one of higher floors
        	for (int i = curFloor + 1; i <= Elevator.numFloors; i++) {
        		if ((c = isCalled(i)).isValid() && canCommit[i]) {
        			return c;
        		}
        	}
        	return new CallRequest();
        }
        
        /* Return true if called at a floor below the current floor or down by curFloor */
        public CallRequest closestCallBelow(int curFloor, boolean[] canCommit) {
        	// Called by current floor down, return true
        	CallRequest c;
        	// Called by one of lower floors
        	for (int i = curFloor - 1; i >= 1; i--) {
        		if ((c = isCalled(i)).isValid() && canCommit[i]) {
        			return c;
        		}
        	}
        	return new CallRequest();
        }
        
        private CallRequest isCalled(int floor) {
        	CallRequest downCalled = isCalled(floor, Direction.DOWN);
        	CallRequest upCalled = isCalled(floor, Direction.UP);
        	
        	if (downCalled.isValid()) {
        		return downCalled;
        	} else if  (upCalled.isValid()) {
        		return upCalled;
        	} else {
        		return new CallRequest();
        	}
        }
        
        public CallRequest isCalled(int floor, Direction direction) {
        	boolean backCalled = isCalled(floor, direction, Hallway.BACK);
        	boolean frontCalled = isCalled(floor, direction, Hallway.FRONT);
        	
        	if (backCalled && frontCalled) {
        		return new CallRequest(floor, direction, Hallway.BOTH);
        	} else if (backCalled) {
        		return new CallRequest(floor, direction, Hallway.BACK);
        	} else if (frontCalled) {
        		return new CallRequest(floor, direction, Hallway.FRONT);
        	} else {
        		return new CallRequest();
        	}
        }
        
        private boolean isCalled(int floor, Direction direction, Hallway hallway) {
            int Index = ReplicationComputer.computeReplicationId(floor, hallway, direction);
            BooleanCanPayloadTranslator translator = translatorArray.get(Index);

            if (translator == null) {
                return false;
            }
            return translator.getValue();
        }

        private void CreateTranslator(int floor, Hallway hallway, Direction direction) {
            int Index = ReplicationComputer.computeReplicationId(floor, hallway, direction);
            ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + Index);
            BooleanCanPayloadTranslator translator = new BooleanCanPayloadTranslator(m);
            this.conn.registerTimeTriggered(m);
            this.translatorArray.put(Index, translator);
        }
    }

    public static class CarCallArray {
        private CANNetwork.CanConnection conn;
        public HashMap<Integer, BooleanCanPayloadTranslator> translatorArray = new HashMap<Integer, BooleanCanPayloadTranslator>();
        public CarCallArray(CANNetwork.CanConnection conn){
            this.conn = conn;

            for (int i = 1; i <= 8; i++) {
                switch (i) {
                    case 1:
                        CreateTranslator(i, Hallway.FRONT);
                        CreateTranslator(i, Hallway.BACK);
                        break;
                    case 2:
                        CreateTranslator(i, Hallway.BACK);
                        break;
                    case 7:
                        CreateTranslator(i, Hallway.FRONT);
                        CreateTranslator(i, Hallway.BACK);
                        break;
                    default:
                        CreateTranslator(i, Hallway.FRONT);
                        break;
                }

            }
        }
        
		public CallRequest closestCallBelow(int curFloor, boolean[] canCommit) {
			//XXX: Potentially include curFloor (for case of someone making a car call while at the current floor?)
	       	CallRequest c;
			for (int i = curFloor - 1; i >= 1; i--) {
        		if ((c = isCalled(i)).isValid() && canCommit[i]) {
        			return c;
        		}
        	}
			return new CallRequest();
		}
		
		public CallRequest closestCallAbove(int curFloor, boolean[] canCommit) {
			//XXX: Potentially include curFloor (for case of someone making a car call while at the current floor?)
	       	CallRequest c;
			for (int i = curFloor + 1; i <= Elevator.numFloors; i++) {
        		if ((c = isCalled(i)).isValid() && canCommit[i]) {
        			return c;
        		}
        	}
			return new CallRequest();
		}
        
        private CallRequest isCalled(int floor) {
        	boolean backCalled = isCalled(floor, Hallway.BACK);
        	boolean frontCalled = isCalled(floor, Hallway.FRONT);
        	
        	if (backCalled && frontCalled) {
        		return new CallRequest(floor, Direction.STOP, Hallway.BOTH);
        	} else if (backCalled) {
        		return new CallRequest(floor, Direction.STOP, Hallway.BACK);
        	} else if (frontCalled) {
        		return new CallRequest(floor, Direction.STOP, Hallway.FRONT);
        	} else {
        		return new CallRequest();
        	}
        }
        
        private boolean isCalled(int floor, Hallway hallway) {
            int Index = ReplicationComputer.computeReplicationId(floor, hallway);
            BooleanCanPayloadTranslator translator = translatorArray.get(Index);

            if (translator == null) {
                return false;
            }
            return translator.getValue();
        }


        private void CreateTranslator(int floor, Hallway hallway) {
            int Index = ReplicationComputer.computeReplicationId(floor, hallway);
            ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + Index);
            BooleanCanPayloadTranslator translator = new BooleanCanPayloadTranslator(m);
            this.conn.registerTimeTriggered(m);
            this.translatorArray.put(Index, translator);
        }
    }
    
    public static class CallRequest {
    	public int floor;
    	public Direction direction;
    	public Hallway hallway;
    	private boolean valid;
    	
    	public CallRequest(int floor, Direction direction, Hallway hallway) {
    		this.floor = floor;
    		this.direction = direction;
    		this.hallway = hallway;
    		this.valid = true;
    	}
    	
    	public CallRequest() {
    		this.valid = false;
    	}
    	
    	public boolean isValid() {
    		return this.valid;
    	}
    }
}
