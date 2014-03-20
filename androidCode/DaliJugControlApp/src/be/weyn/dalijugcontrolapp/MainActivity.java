package be.weyn.dalijugcontrolapp;

import java.io.IOException;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends Activity implements SensorEventListener, OnTouchListener  {
	
	public static int J1xMax = 25;  //max turning
    public static int J1yMax = 65; //65;  //max walking gait length per step, [mm]
    public static int J2xMax = 10;  //max tilt from side to side, [mm]
    public static int J2yMax = 10;  //max tilt from front to back, [mm]
    public static int J3xMax = 50;  //max translation side to side, [mm] *WARNING* cannot be larger than RMPosX!! Will cause serious damage to robot.
    public static int J3yMax = 50;  //max translation front to back, [mm]
    public static int J1zMax = 50;  //max crab walk step length [mm]
    public static int J2zMax = 100;  //max body height [mm]
    public static int J3zMax = 15;  //max twist 
    public static int J4yMax = 90;  //max pan
    public static int J4xMax = 90;  //max tilt
    public static int J4speedMax = 2000; //max move speed [ms]
    public static int J4speedMin = 10; //min move speed [ms]
    
    private static int BodyPosY = 0;
    private static int BodyPosX = 0;
    private static int BodyPosZ = 0;
    private static int BodyRotY = 0;
    private static int BodyRotX = 0;
    private static int BodyRotZ = 0;
    private static int TravelLengthX = 0; 
    private static int TravelLengthZ = 0;
    private static int TravelRotationY = 0;
    private static int TravelSpeed = 2000;
    
    private static int GaitType = 0;
    private static String[] GaiteName = {"Ripple", "Tripod", "Wave"};  
    private static boolean OnRoad = true;

	private SensorManager mSensorManager;
	  private float[] gravity = new float[3]; 
	  
	  // TODO: autoscale
	  /* nexus 5
	  public float radius = 200;
	  public float[] centerMotion = {220, 400};	  
	  public float[] centerTilt = {700, 400};	  
	  public float[] centerTranslate = {220, 890};	
	  */
	  // samsung
	  public float radius = 100;
	  public float[] centerMotion = {120, 180};	  
	  public float[] centerTilt = {350, 180};	  
	  public float[] centerTranslate = {120, 450};	
	  private boolean motionTriggered = false;
	  private boolean tiltTriggered = false;
	  private boolean translateTriggered = false;
	  private String previousCommand = "";
	  
	  Circle motionJoystick;
	  Circle motionArea;
	  Circle tiltJoystick;
	  Circle tiltArea;
	  Circle translateJoystick;
	  Circle translateArea;
	  TextView motionInfo;
	  TextView gaitInfo;
	  TextView onRoadInfo;
	  SeekBar crabSeekBar;
	  SeekBar speedSeekBar;
	  SeekBar heightSeekBar;
	  SeekBar rotateSeekBar;
	  
	  BluetoothActivity bluetooth;
	  Thread workerThread;
	  boolean stopWorker = false;
	  boolean sleep = true;
	  
	  @Override
	  protected void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
	        
	    // Get an instance of the SensorManager
	    mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
	    
	    setContentView(R.layout.activity_main);
	    
	    FrameLayout main = (FrameLayout) findViewById(R.id.main_view);
	    motionArea = new Circle(this, centerMotion[0], centerMotion[1], radius, 0xFF000000, true, "Walk / Turn");
	    motionArea.setOnTouchListener(this);
        main.addView(motionArea);
        
        motionJoystick = new Circle(this, centerMotion[0],  centerMotion[1], 20, 0xFFFF0000, false, "");
        main.addView(motionJoystick);
        
        tiltArea = new Circle(this, centerTilt[0], centerTilt[1], radius, 0xFF000000, true, "Tilt X/Y");
	    tiltArea.setOnTouchListener(this);
        main.addView(tiltArea);
        
        tiltJoystick = new Circle(this, centerTilt[0],  centerTilt[1], 20, 0xFFFF0000, false, "");
        main.addView(tiltJoystick);
        
        translateArea = new Circle(this, centerTranslate[0], centerTranslate[1], radius, 0xFF000000, true, "Translate X/Y");
        translateArea.setOnTouchListener(this);
        main.addView(translateArea);
        
        translateJoystick = new Circle(this, centerTranslate[0],  centerTranslate[1], 20, 0xFFFF0000, false, "");
        main.addView(translateJoystick);
        
        motionInfo = (TextView) findViewById(R.id.motionText);
        motionInfo.setText("0 / 0");
        motionInfo.setKeyListener(null);
        
        gaitInfo = (TextView) findViewById(R.id.gaitText);
        gaitInfo.setText(GaiteName[GaitType]);
        gaitInfo.setKeyListener(null);
        
        onRoadInfo = (TextView) findViewById(R.id.onroadButton);
        
        crabSeekBar = (SeekBar) findViewById(R.id.crabBar);
        crabSeekBar.setEnabled(false);
        crabSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                    boolean fromUser) {
            	TravelLengthX = (int) (((progress - 50) / 50.0) * J1zMax);
            }

			@Override
			public void onStartTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onStopTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}
         });
        
        speedSeekBar = (SeekBar) findViewById(R.id.speedBar);
        speedSeekBar.setEnabled(false);
        speedSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                    boolean fromUser) {
            	TravelSpeed = J4speedMin + (int) ((progress / 100.0) * (J4speedMax - J4speedMin));
            	motionInfo.setText(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed));
            }

			@Override
			public void onStartTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onStopTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}
         });
        
        heightSeekBar = (SeekBar) findViewById(R.id.heightBar);
        heightSeekBar.setEnabled(false);
        heightSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                    boolean fromUser) {
            	BodyPosY = (int) ((progress / 100.0) * J2zMax);
            	motionInfo.setText(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed));
            }

			@Override
			public void onStartTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onStopTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}
         });
        
        rotateSeekBar = (SeekBar) findViewById(R.id.rotateBar);
        rotateSeekBar.setEnabled(false);
        rotateSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                    boolean fromUser) {
            	BodyRotY = (int) (((progress - 50)  / 50.0) * J3zMax);
            	motionInfo.setText(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed));
            }

			@Override
			public void onStartTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onStopTrackingTouch(SeekBar arg0) {
				// TODO Auto-generated method stub
				
			}
         });
        
        bluetooth = new BluetoothActivity();
        
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        
        
        workerThread = new Thread(new Runnable()
        {
            public void run()
            {                
               while(!Thread.currentThread().isInterrupted() && !stopWorker)
               {
                    sendData();                   

					try {
					    Thread.sleep(100);
					} catch (InterruptedException e) {}
               }
            }
        });

        
	  }
	  
	  @Override
	  public void onConfigurationChanged(Configuration newConfig) {
	      super.onConfigurationChanged(newConfig);
	      setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
	  }

	  @Override
	  protected void onResume() {
	    super.onResume();
	    mSensorManager.registerListener(
	      this, 
	      mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
	       SensorManager.SENSOR_DELAY_NORMAL );
	  }

	  @Override
	  protected void onPause() {
	    super.onPause();
	    mSensorManager.unregisterListener(this);
	  }
	  
	  @Override
		public boolean onTouch(View v, MotionEvent event) {
		  if (event.getAction() != MotionEvent.ACTION_DOWN || sleep)
		  {
		      return false;
		  }
		  
		    int _x = (int) event.getX();
		    int _y = (int) event.getY();
		    
		    double distance = Math.sqrt(((centerMotion[0]-_x)*(centerMotion[0]-_x)) + ((centerMotion[1]-_y)*(centerMotion[1]-_y))); 
		    
		    //Log.w("be.weyn.dalijugcontrolapp", "Circle touch: " + distance);
		    
		    if (distance < radius)
		    {
		    	motionTriggered = !motionTriggered;
		    	
//		    	if (!motionTriggered)
//		    	{
//		    		gravity[0] = 0;
//		    		gravity[1] = 0.3f * 9.6f;
//		    		updateMotionCircle();
//		    	}
		    	return true;
		    }
		    else
		    {
		    	distance = Math.sqrt(((centerTilt[0]-_x)*(centerTilt[0]-_x)) + ((centerTilt[1]-_y)*(centerTilt[1]-_y)));
		    	if (distance < radius)
			    {
			    	tiltTriggered = !tiltTriggered;
			    	
//			    	if (!tiltTriggered)
//			    	{
//			    		gravity[0] = 0;
//			    		gravity[1] = 0.3f * 9.6f;
//			    		updateTiltCircle();
//			    	}
			    	return true;
			    }
			    else
			    {
			    	distance = Math.sqrt(((centerTranslate[0]-_x)*(centerTranslate[0]-_x)) + ((centerTranslate[1]-_y)*(centerTranslate[1]-_y)));
			    	if (distance < radius)
				    {
				    	translateTriggered = !translateTriggered;
				    	
//				    	if (!tiltTriggered)
//				    	{
//				    		gravity[0] = 0;
//				    		gravity[1] = 0.3f * 9.6f;
//				    		updateTiltCircle();
//				    	}
				    	return true;
				    }
				    else
				    {
				    	return false;
				    }
			    }
		    }
		} 

	  public void onSensorChanged(SensorEvent evt) {
	    int type=evt.sensor.getType();
	    
	    if (type == Sensor.TYPE_ACCELEROMETER) {
	      gravity[0]=(gravity[0]*2+evt.values[0])*0.33334f;
	      gravity[1]=(gravity[1]*2+evt.values[1])*0.33334f;
	      gravity[2]=(gravity[2]*2+evt.values[2])*0.33334f;
	    }
	    
	    if (motionTriggered)  updateMotionCircle();
	    if (tiltTriggered)  updateTiltCircle();
	    if (translateTriggered)  updateTranslateCircle();
	  }
	  
	  private void updateMotionCircle() {		  
		float rotation =  (-gravity[0] / 9.6f);
		float walking = 0.3f + (-gravity[1] / 9.6f);
		TravelRotationY = (int) (-rotation * J1xMax);
		TravelLengthZ = (int) (walking * J1yMax);
		
		motionInfo.setText(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed));
		
		motionJoystick.Move(centerMotion[0] + rotation * radius * 0.9f, centerMotion[1]+ walking * radius * 0.9f);
		
		try {
		    Thread.sleep(1);
		} catch (InterruptedException e) {}
	  }
	  
	  private void updateTiltCircle() {		  
			float tiltX =  (-gravity[0] / 9.6f);
			float tiltY = -0.3f + (gravity[1] / 9.6f);
			BodyRotZ = (int) (-tiltX * J2xMax);
			BodyRotX = (int) (tiltY * J2yMax);
			
			motionInfo.setText(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed));
			
			tiltJoystick.Move(centerTilt[0] + tiltX * radius * 0.9f, centerTilt[1]+ tiltY * radius * 0.9f);
			
			try {
			    Thread.sleep(1);
			} catch (InterruptedException e) {}
		}
	  
	  private void updateTranslateCircle() {		  
			float translateX =  (-gravity[0] / 9.6f);
			float translateY = -0.3f + (gravity[1] / 9.6f);
			BodyPosX = (int) (-translateX * J2xMax);
			BodyPosZ = (int) (translateY * J2yMax);
			
			motionInfo.setText(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed));
			
			translateJoystick.Move(centerTranslate[0] + translateX * radius * 0.9f, centerTranslate[1]+ translateY * radius * 0.9f);
			
			try {
			    Thread.sleep(1);
			} catch (InterruptedException e) {}
		}
	  
	@SuppressLint("DefaultLocale")
	private void sendData()
	{
		String command = String.format("motion=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;\n", BodyPosY, BodyPosX, BodyPosZ, BodyRotY, BodyRotX, BodyRotZ, TravelLengthX, TravelLengthZ, TravelRotationY, TravelSpeed);
		if (bluetooth != null && command != previousCommand)
		{
			try {
				bluetooth.sendData(command);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			previousCommand = command;
		}
		
	}

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	  }
	
	public void ConnectBT(View view)
	{
		if (bluetooth != null)
		{
			boolean findBT = bluetooth.findBT();
			if (findBT)
			{
				try {
					bluetooth.openBT();		
					workerThread.start();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					Toast.makeText(MainActivity.this, "Cannot connect to DaliJug", Toast.LENGTH_SHORT).show();
				}
			} else {
				Toast.makeText(MainActivity.this, "Bluetooth not enabled or cannot find DaliJug", Toast.LENGTH_SHORT).show();
			}
		} else
		{
			Toast.makeText(MainActivity.this, "Bluetooth is not enabled.", Toast.LENGTH_SHORT).show();
		}
	}
	
	public void StopMotion(View view)
	{
		motionTriggered = false;
		tiltTriggered = false;
		translateTriggered = false;
		
		gravity[0] = 0;
		gravity[1] = 0.3f * 9.6f;
		updateMotionCircle();
		updateTiltCircle();	
		updateTranslateCircle();	
		
		crabSeekBar.setProgress(50);
		rotateSeekBar.setProgress(50);
	}
	
	public void WakeSleep(View view)
	{
		sleep = !sleep;
		
		Button wakeButton = (Button) findViewById(R.id.wakeButton);
		if (sleep)
		{
			wakeButton.setText("Wake");
			BodyPosY = 0;
			heightSeekBar.setProgress(0);

			speedSeekBar.setEnabled(false);
			crabSeekBar.setEnabled(false);
			rotateSeekBar.setEnabled(false);
			heightSeekBar.setEnabled(false);
			
			try {
				bluetooth.sendData("motion=sleep;\n");
			} catch (Exception e) {
				Toast.makeText(MainActivity.this, "Cannot set DaliJug to sleep", Toast.LENGTH_SHORT).show();
			}
			
		
		} else {
			wakeButton.setText("Sleep");
			BodyPosY = 50;
			heightSeekBar.setProgress((int) (100 * (50.0 / J2zMax)));
			
			speedSeekBar.setEnabled(true);
			crabSeekBar.setEnabled(true);
			rotateSeekBar.setEnabled(true);
			heightSeekBar.setEnabled(true);
			
			try {
				bluetooth.sendData("motion=wake;\n");
			} catch (Exception e) {
				Toast.makeText(MainActivity.this, "Cannot wake DaliJug", Toast.LENGTH_SHORT).show();
			}
		}
		
	}
	
	public void ChangeGait(View view)
	{
		GaitType++;
		if (GaitType > 2) GaitType = 0;

        gaitInfo.setText(GaiteName[GaitType]);
        try {
			bluetooth.sendData("gaittype=" + GaitType + ";\n");
		} catch (Exception e) {
			Toast.makeText(MainActivity.this, "Cannot chnage gait DaliJug", Toast.LENGTH_SHORT).show();
		}
	
	}
	
	public void ChangeOnRoad(View view)
	{
		OnRoad = !OnRoad;
        onRoadInfo.setText((OnRoad ? "On Road" : "Off Road") + OnRoad);
        
        try {
			bluetooth.sendData("onroad=" + OnRoad + ";\n");
		} catch (Exception e) {
			Toast.makeText(MainActivity.this, "Cannot chnage gait DaliJug", Toast.LENGTH_SHORT).show();
		}
	}
}
			