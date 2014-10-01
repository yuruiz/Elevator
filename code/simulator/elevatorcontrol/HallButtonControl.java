package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class HallButtonControl extends Controller {
	
    /***************************************************************************
     * Declarations
     **************************************************************************/
	
    //local physical state
    private ReadableHallCallPayload localHallCall;
    private WriteableHallLightPayload localHallLight;
    
    //network interface
    private WriteableCanMailbox networkHallLightOut;
    // translator for the hall light message -- this is a generic translator
    private BooleanCanPayloadTranslator mHallLight;
    
    // network interface
    private WriteableCanMailbox networkHallCall;
    private BooleanCanPayloadTranslator mHallCall;

    //received door closed message
    private ReadableCanMailbox networkDoorClosedLeft;
    private ReadableCanMailbox networkDoorClosedRight;
    
    //translator for the doorClosed message -- this translator is specific
    //to this messages, and is provided the elevatormodules package
    private DoorClosedCanPayloadTranslator mDoorClosedLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedRight;
    
    //translator for the AtFloor message -- this translator is specific
    //to this message and is provided elevatormodules package
    private ReadableCanMailbox networkAtFloor;
    private AtFloorCanPayloadTranslator mAtFloor;
    
    //translator for the DesiredFloor message -- this translator is specific
    //to this message and is provided elevatormodules package
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;
    
    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_IDLE,
        STATE_ACTIVE
    }
    
    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_IDLE;

    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
    public HallButtonControl(SimTime period, int floor, Hallway hallway, Direction direction, boolean verbose) {
        //call to the Controller superclass constructor is required
        super(getReplicationName(), verbose);
        
        //stored the constructor arguments in internal state
        this.period = period;
        this.hallway = hallway;
        this.direction = direction;
        this.floor = floor;
        
        /* 
         * The log() method is inherited from the Controller class.  It takes an
         * array of objects which will be converted to strings and concatenated
         * only if the log message is actually written.  
         */
        log("Created HallButtonLight with period = ", period);

        /* 
         * Create readable payloads and translators for all messages in input interface
         * 		- mAtFloor[f,b,d], mDesiredFloor, mDoorClosed[b,r], HallCall[f,b,d]
         */
       
        // mAtFloor
        networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID);
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
        
        // mDesiredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        
        // mDoorClosedLeft, mDoorClosedRight
        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        mDoorClosedLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedLeft, hallway, Side.LEFT);
        mDoorClosedRight = new DoorClosedCanPayloadTranslator(networkDoorClosedRight, hallway, Side.RIGHT);

        // HallCall
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);

        // Register input messages periodically
        canInterface.registerTimeTriggered(networkAtFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDoorClosedLeft);
        canInterface.registerTimeTriggered(networkDoorClosedRight);
        physicalInterface.registerTimeTriggered(localHallCall);
        
        
        /* 
         * Create writeable payloads for all messages in output interface
         * 		- HallLight[f,b,d], mHallLight[f,b,d], mHallCall[f,b,d]
         */
        
        // HallLight
        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);
        
        // mHallLight
        networkHallLightOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallLight = new BooleanCanPayloadTranslator(networkHallLightOut);
        
        // mHallCall
        networkHallCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID);
        mHallCall = new BooleanCanPayloadTranslator(networkHallCall);
        
        /* Send/broadcast output messages periodically */
        physicalInterface.sendTimeTriggered(localHallLight, period);
        canInterface.sendTimeTriggered(networkHallLightOut, period);
        canInterface.sendTimeTriggered(networkHallCall, period);

        // Start the periodic timer
        timer.start(period);
    }

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
        switch(state) {
            case STATE_IDLE:
            	localHallLight.set(false);
            	mHallLight.set(false);
            	mHallCall.set(false);
            	//#transition HBC.T.1
            	if (localHallCall.pressed()) {
            		newState = State.STATE_ACTIVE;
            	}
                break;
            case STATE_ACTIVE:
            	localHallLight.set(true);
            	mHallLight.set(true);
            	mHallCall.set(true);
            	//#transition HBC.T.2
            	if (mAtFloor.getValue() && mDesiredFloor.getDirection().equals(direction) 
            							&& !localHallCall.pressed()) {
            		newState = State.STATE_IDLE;
            	}
                break;
        }
    	log(getReplicationName() + ": " + state.toString() + " -> " + newState.toString());
        state = newState;

        timer.start(period);
	}
	
	private String getReplicationName() {
		return "HallButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway, direction);
	}

}
