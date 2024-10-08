package ai.flow.android.sensor;

import ai.flow.definitions.Definitions;
import ai.flow.sensor.SensorInterface;
import messaging.ZMQPubHandler;
import org.capnproto.PrimitiveList;

import java.util.Arrays;

import ai.flow.definitions.Definitions;
import ai.flow.definitions.MessageBase;

class MsgDeviceState extends MessageBase {

    public Definitions.DeviceState.Builder deviceStateEvent;

    public MsgDeviceState() {
        super();
        initFields();
        bytesSerializedForm = computeSerializedMsgBytes();
        initSerializedBuffer();
    }

    private void initFields(){
        event = messageBuilder.initRoot(Definitions.Event.factory);
        deviceStateEvent = event.initDeviceState();
        deviceStateEvent.setThermalStatus(ai.flow.definitions.Definitions.DeviceState.ThermalStatus.YELLOW);
    }
}

public class DeviceStateManager extends SensorInterface implements Runnable{

    public ZMQPubHandler ph;
    public String topic;
    public boolean initialized = false;
    public boolean running = false;
    public int frequency;
    public Thread thread;
    public int delay; // in milliseconds;
    public MsgDeviceState msgDeviceState = new MsgDeviceState();

    public DeviceStateManager(int frequency) {
        ph = new ZMQPubHandler();
        ph.createPublishers(Arrays.asList("deviceState"));
        this.frequency = frequency;
        this.delay = (int) (1.0f / frequency * 1000);

    }

    public void start() {
        if (thread == null) {
            thread = new Thread(this, "deviceStateManager");
            thread.setDaemon(false);
            thread.start();
        }
    }

    public void run(){
        if (running)
            return;
        initialized = true;
        running = true;

        while (running){
            ph.publishBuffer("deviceState", msgDeviceState.serialize(true));
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void stop() {
        if (!running)
            return;
        initialized = false;
        running = false;
        try {
            thread.join();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        thread = null;
    }

    public void dispose(){
        stop();
    }

    public boolean isRunning(){
        return running;
    }
    public boolean isInitialized(){
        return initialized;
    }
}
