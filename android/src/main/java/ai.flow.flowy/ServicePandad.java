package ai.flow.flowy;
//Java_org_jagheterfredrik_flowapp_ServicePandad_nativeInit

import android.app.Service;
import android.content.Context;
import android.os.IBinder;
import android.content.Intent;
import android.util.Log;

public class ServicePandad extends Service implements Runnable {

    private static final String TAG = "ServicePandad";
    // Thread for Python code
    private Thread applicationThread = null;
    private int fd;

    public int startType() {
        return START_NOT_STICKY;
    }

    @Override
    public IBinder onBind(Intent arg0) {
        return null;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        System.loadLibrary("pandad");
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        if (applicationThread != null) {
            Log.v("python service", "service exists, do not start again");
            return startType();
        }
        this.fd = intent.getIntExtra("fd", 0);
        System.out.println("I was rang with " + this.fd);

        applicationThread = new Thread(this);
        applicationThread.start();

        System.out.println("Service pid is " + android.os.Process.myPid());
        return startType();
    }

    public static void start(Context ctx, int fd) {
        Intent serviceIntent = new Intent(ctx, ServicePandad.class);
        serviceIntent.putExtra("fd", fd);
        ctx.startService(serviceIntent);
    }

    @Override
    public void run(){
        nativeInit(this.fd);
    }

    // Native part
    public static native void nativeInit(int fd);
}
