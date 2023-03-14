package overhorizon.mobiledepthfinder;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.ToggleButton;

interface MyFun {
	public void use(int arg);
}

public class DepthFinder extends Activity {

	MyProgressDialog progress;
	private MyCamera cam;
	private Handler handler;
	private GestureDetector gesture;
	Boolean isReconstructed;
	Boolean managerConnected = false;

	enum State {
		PLAYING, RECORDING, PRESENTING
	};

	State appState;

	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				System.out.println("OpenCV loaded successfully");
				managerConnected = true;
				startMyApp();
			}
				break;
			default: {
				System.out.println("Cannot connect to OpenCV Manager");
				super.onManagerConnected(status);
				DepthFinder.this.finish();
			}
				break;
			}
		}

	};

	@Override
	public void onCreate(Bundle savedInstanceState) {

		super.onCreate(savedInstanceState);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.activity_depthfinder);
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_5, this,
				mLoaderCallback);
	}

	private void startMyApp() {

		handler = new Handler() {

			@Override
			public void handleMessage(Message msg) {
				Box.State myState = Box.State.values()[msg.what];
				switch (myState) {
				case IMAGE_GRABBED:
					ImageView view = (ImageView) findViewById(R.id.imageView);
					view.setImageBitmap(cam.cameraView);
					break;
				case EXIT:
					DepthFinder.this.finish();
					break;
				case IMAGE_ADDED:
					TextView fv = (TextView) findViewById(R.id.frames);
					fv.setText("frames: " + Box.imagesList.size());
					break;
				case UPDATE_VIEW:
					ImageView viewReconstructed = (ImageView) findViewById(R.id.imageView);
					viewReconstructed.setImageBitmap(MyProjection.resultImage);
					TextView angleView = (TextView) findViewById(R.id.xAngle);
					angleView.setText("" + MyProjection.xAngle.intValue());
					angleView = (TextView) findViewById(R.id.yAngle);
					angleView.setText("" + MyProjection.yAngle.intValue());
					TextView posView = (TextView) findViewById(R.id.depth);
					posView.setText("depth: " + MyProjection.zPos.intValue());
					break;
				case RECONSTRUCTED:
					Button b = (Button) findViewById(R.id.button_zoomin);
					b.setEnabled(true);
					b = (Button) findViewById(R.id.button_zoomout);
					b.setEnabled(true);
					appState = State.PRESENTING;

					break;
				case FAILURE:
					new MyMessageBox(
							DepthFinder.this,
							"Couldn't reconstruct from this, please try again.",
							new MyFun() {

								@Override
								public void use(int arg) {
									restart();

								}
							}, null);
					break;
				}
			}

		};

	
		// ---------------gesture listener
		gesture = new GestureDetector(this, new MyGestureDetector(handler,
				DepthFinder.this));
		ImageView viewReconstructed = (ImageView) findViewById(R.id.imageView);

		viewReconstructed.setOnTouchListener(new OnTouchListener() {

			public boolean onTouch(View v, final MotionEvent event) {
				if (event.getPointerCount() > 1)
					return true;

				if (appState == State.PRESENTING) {
					gesture.onTouchEvent(event);
				}

				return true;
			}

		});


		// -------------------reduction seekbar listener
		VerticalSeekBar vsb = (VerticalSeekBar) findViewById(R.id.reductionBar);
		vsb.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
			}

			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				TextView colors = (TextView) findViewById(R.id.reduction);
				colors.setText("colours: " + progress);
				MyCamera.reduction = progress;
			}
		});

		
		// -------------method spinner listener
		Spinner sp = (Spinner) findViewById(R.id.spinner1);
		ArrayAdapter<String> adapter = new ArrayAdapter<String>(
				DepthFinder.this, android.R.layout.simple_spinner_item,
				MyFeatures.Methods.getStrings());
		adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		sp.setAdapter(adapter);
		sp.setOnItemSelectedListener(new OnItemSelectedListener() {

			@Override
			public void onItemSelected(AdapterView<?> parent, View view,
					int pos, long id) {
				MyFeatures.method = MyFeatures.Methods.values()[pos];

			}

			@Override
			public void onNothingSelected(AdapterView<?> arg0) {
			}

		});

		// -------------filter button listener
		ToggleButton tb = (ToggleButton) findViewById(R.id.button_filter);
		tb.setChecked(false);
		MyCamera.filter = false;
		tb.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				if (((ToggleButton) v).isChecked()) {
					MyCamera.filter = true;
				} else {
					MyCamera.filter = false;
				}

			}
		});
		
		
		// -------------zoomin/out button listener
		Button b = (Button)findViewById(R.id.button_zoomin);
		b.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				MyProjection.zPos -= 1;
				MyProjection.getResults(handler);
				
			}
		});

		b = (Button)findViewById(R.id.button_zoomout);
		b.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				MyProjection.zPos += 1;
				MyProjection.getResults(handler);
				
			}
		});

		
		// ---------------starting camera preview
		restart();

	}

	

	// -------------record button listener
	public void record_button(View view) {

		final Button b = (Button) view;

		if (appState == State.PLAYING) {

			appState = State.RECORDING;
			b.setText("stop");
			Spinner sp = (Spinner) findViewById(R.id.spinner1);
			sp.setEnabled(false);
			VerticalSeekBar vsb = (VerticalSeekBar) findViewById(R.id.reductionBar);
			vsb.setEnabled(false);
			ToggleButton tb = (ToggleButton) findViewById(R.id.button_filter);
			tb.setEnabled(false);

		} else if (appState == State.RECORDING) {
			if (Box.imagesList.size() < 2)
				return;
			cam.isPlaying = false;
			cam.stop();
			new MyMessageBox(this, "reconstruct?", new MyFun() {

				@Override
				public void use(int arg) {
					startReconstructing();
					b.setText("new");

				}
			}, new MyFun() {

				@Override
				public void use(int arg) {
					restart();
				}
			});

		} else if (appState == State.PRESENTING) {
			restart();
		}
	}

	private void restart() {

		Spinner sp = (Spinner) findViewById(R.id.spinner1);
		sp.setEnabled(true);
		VerticalSeekBar vsb = (VerticalSeekBar) findViewById(R.id.reductionBar);
		vsb.setEnabled(true);

		Button b = (Button) findViewById(R.id.button_capture);
		b.setText("record");
		b = (ToggleButton) findViewById(R.id.button_filter);
		b.setEnabled(true);

		b = (Button) findViewById(R.id.button_zoomin);
		b.setEnabled(false);
		b = (Button) findViewById(R.id.button_zoomout);
		b.setEnabled(false);

		TextView tv = (TextView) findViewById(R.id.xAngle);
		tv.setText("");
		tv = (TextView) findViewById(R.id.yAngle);
		tv.setText("");
		tv = (TextView) findViewById(R.id.reduction);
		tv.setText("colours: " + vsb.getProgress());

		tv = (TextView) findViewById(R.id.depth);
		tv.setText("depth: " + "0");// MyProjection.zPos.intValue());

		TextView fv = (TextView) findViewById(R.id.frames);
		fv.setText("frames: 0");

		Box.clear();
		appState = State.PLAYING;
		startMyCamera();
	}

	
	private void startMyCamera() {

		(new Thread(new Runnable() {

			@Override
			public void run() {
				cam = new MyCamera(DepthFinder.this);
				Box.focal = cam.getFocal();
				cam.play(handler);
			}
		})).start();
	}
		
	
	private void startReconstructing() {
		progress = new MyProgressDialog(this, handler);
		progress.show();

		(new Thread(new Runnable() {

			@Override
			public void run() {

				Box recon = new Box(progress, handler);
				recon.start();

			}
		})).start();

	}

	protected void onRestart() {
		super.onRestart();
		restart();
	}

	public void onPause() {

		if (cam != null)
			cam.stop();
		super.onPause();
	}

}
