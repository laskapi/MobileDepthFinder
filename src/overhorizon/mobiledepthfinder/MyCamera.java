package overhorizon.mobiledepthfinder;

import java.util.List;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.hardware.Camera;
import android.os.Handler;
import android.util.Log;

public class MyCamera {

	static private float focal;
	static private Camera.Size resolution;

	private Handler handler;

	final static int FRAME_INTERVAL = 1000;
	public Context mContext;

	volatile Boolean isRecording, isPlaying;
	Bitmap cameraView;
	VideoCapture cam;
	static int reduction;
	protected static boolean filter;

	static {
		Camera camera = null;
		try {
			camera = Camera.open(); // attempt to get a Camera instance
			focal = camera.getParameters().getFocalLength();
			// setResolution(camera);
			camera.release();

		} catch (Exception e) {
			Log.e(Box.TAG, "Couldn't open camera: " + e.getMessage());

		}

	}

	public MyCamera(Context c) {
		mContext = c;
		reduction = ((VerticalSeekBar) ((DepthFinder) c)
				.findViewById(R.id.reductionBar)).getProgress();

		isPlaying = true;
		isRecording = false;
		cam = new VideoCapture();
	}

	public Camera.Size getResolution() {
		return resolution;
	}

	public float getFocal() {
		return focal;
	}

	// ---------------------------find and set optimal camera resolution
	static private void setResolution(Camera camera) {
		Camera.Parameters cameraParams = camera.getParameters();
		List<Camera.Size> supportedPreviewSizes = cameraParams
				.getSupportedPreviewSizes();
		resolution = supportedPreviewSizes.get(0);

		for (Camera.Size s : supportedPreviewSizes) {
			if (s.width > resolution.width)
				resolution = s;
		}

		cameraParams.setPreviewSize(resolution.width, resolution.height);
		camera.setParameters(cameraParams);

	}

	Boolean play(Handler handler) {
		this.handler = handler;
		cam.open(0);
		// cam.set(3, resolution.height);
		// cam.set(4, resolution.width);

		Mat a = new Mat();
		Mat b = new Mat();

		long old, actual;
		old = System.currentTimeMillis();
		while (isPlaying) {

			// --test time
			actual = System.currentTimeMillis();
			if (actual - old > FRAME_INTERVAL) {
				old = actual; // test time

				boolean grabbed = cam.grab();
				if (grabbed) {

					cam.retrieve(a);
					Imgproc.cvtColor(a, b, Imgproc.COLOR_BGR2GRAY);
					if (filter) {
						Imgproc.medianBlur(b, a, 5);
					}
					b = a;
					reduce(b, reduction);
					// here to un/comment for clear/featured preview
					Mat featureMat = MyFeatures.getFeature(b);
					// Mat featureMat = b;
					// ---------------------------------
					cameraView = Bitmap.createBitmap(b.cols(), b.rows(),
							Config.ARGB_8888);
					Utils.matToBitmap(featureMat, cameraView, true);
					handler.sendEmptyMessage(Box.State.IMAGE_GRABBED.ordinal());

					if (((DepthFinder) mContext).appState == DepthFinder.State.RECORDING) {

						Box.imagesList.add(b.clone());

						handler.sendEmptyMessage(Box.State.IMAGE_ADDED
								.ordinal());

					}
				}
			}

		}
		if (cam != null) {
			cam.release();
			cam = null;
		}
		return true;
	}

	private void reduce(Mat image, int div) {
		if (div < 1)
			div = 1;
		div = 256 - div;
		int[] lut = new int[256];
		for (int i = 0; i < 256; i++) {
			lut[i] = (div * (i / div));
		}

		int size = (int) image.total() * image.channels();
		byte[] buff = new byte[size];
		image.get(0, 0, buff);

		for (int i = 0; i < size; i++) {
			buff[i] = (byte) (lut[buff[i] + 128] - 128);

		}

		image.put(0, 0, buff);
	}

	public void stop() {
		isPlaying = false;

	}
}
