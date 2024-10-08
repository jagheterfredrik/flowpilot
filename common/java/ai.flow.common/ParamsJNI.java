package ai.flow.common;

import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;

public class ParamsJNI extends ParamsInterface {

    static {
        System.loadLibrary("params_jni"); // Load your JNI library
    }

    private long nativeHandle;

    public enum ParamKeyType {
        PERSISTENT,
        CLEAR_ON_MANAGER_START,
        CLEAR_ON_ONROAD_TRANSITION,
        CLEAR_ON_OFFROAD_TRANSITION,
        DEVELOPMENT_ONLY,
        ALL
    }

    public ParamsJNI() {
        nativeHandle = createNativeInstance();
    }

    @Override
    protected void finalize() {
        releaseNativeInstance(nativeHandle);
    }

    public void clearAll(ParamKeyType txType) {
        clearAllNative(nativeHandle, txType.ordinal());
    }

    public boolean checkKey(String key) {
        return checkKeyNative(nativeHandle, key);
    }

    public byte[] getBytes(String key, boolean block) {
        checkKey(key);
        return getNative(nativeHandle, key, block);
    }

    public byte[] getBytes(String key) {
        return getBytes(key, false);
    }

    public String get(String key, boolean block) {
        return new String(getBytes(key, block), StandardCharsets.UTF_8);
    }

    public String get(String key) {
        return get(key, false);
    }

    public boolean getBool(String key, boolean block) {
        checkKey(key);
        return getBoolNative(nativeHandle, key, block);
    }

    public boolean getBool(String key) {
        return getBool(key, false);
    }

    public void put(String key, String dat) {
        checkKey(key);
        putNative(nativeHandle, key, dat);
    }

    public void putBool(String key, boolean val) {
        checkKey(key);
        putBoolNative(nativeHandle, key, val);
    }

    public void putNonBlocking(String key, String dat) {
        checkKey(key);
        putNonBlockingNative(nativeHandle, key, dat);
    }

    public void putBoolNonBlocking(String key, boolean val) {
        checkKey(key);
        putBoolNonBlockingNative(nativeHandle, key, val);
    }

    public void remove(String key) {
        checkKey(key);
        removeNative(nativeHandle, key);
    }

    public boolean exists(String key) {
        return getBytes(key).length > 0;
    }

    public String getParamPath(String key) {
        return getParamPathNative(nativeHandle, key);
    }

    public String getParamPath() {
        return getParamPath("");
    }

    public String[] allKeys() {
        return allKeysNative(nativeHandle);
    }

    // Native method declarations
    private native long createNativeInstance();

    private native void releaseNativeInstance(long nativeHandle);

    private native void clearAllNative(long nativeHandle, int txType);

    private native boolean checkKeyNative(long nativeHandle, String key);

    private native byte[] getNative(long nativeHandle, String key, boolean block);

    private native boolean getBoolNative(long nativeHandle, String key, boolean block);

    private native void putNative(long nativeHandle, String key, String dat);

    private native void putBoolNative(long nativeHandle, String key, boolean val);

    private native void putNonBlockingNative(long nativeHandle, String key, String dat);

    private native void putBoolNonBlockingNative(long nativeHandle, String key, boolean val);

    private native void removeNative(long nativeHandle, String key);

    private native String getParamPathNative(long nativeHandle, String key);

    private native String[] allKeysNative(long nativeHandle);
}

class UnknownKeyName extends Exception {
    public UnknownKeyName(String message) {
        super(message);
    }
}

class KeyboardInterrupt extends Exception {
    public KeyboardInterrupt() {
        super();
    }
}