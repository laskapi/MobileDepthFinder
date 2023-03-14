package overhorizon.mobiledepthfinder;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.KeyPoint;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.graphics.Bitmap;
import android.os.Handler;
import android.text.InputFilter;
import android.text.InputType;
import android.text.Spanned;
import android.widget.EditText;

public class Box {

	static final String TAG = "DepthFinder";
	// -------------------
	static ArrayList<Mat> imagesList;
	static ArrayList<List<KeyPoint>> pointsList;
	static ArrayList<ArrayList<KeyPoint>> goodPointsList;
	static Map<MyPair<Integer, Integer>, ArrayList<DMatch>> matchesMap;

	static ArrayList<CloudPoint> pcloud;

	static Map<Integer, Mat> pMats;
	static Mat intrinsicMat;
	static Mat invIntrinsicMat;
	static Mat distortionMat;
	static float focal;
	static ArrayList<KeyPoint> correspImg1Pt;

	static Size size;
	static int m_first_view;
	static int m_second_view;
	static Bitmap source;

	// ---
	public MyProgressDialog progress;
	// ---
	private Handler handler;

	enum State {
		IMAGE_GRABBED, CAMERA_RELEASED, EXIT, RECONSTRUCTED, IMAGE_ADDED, UPDATE_VIEW, FAILURE
	}

	

	public Box(MyProgressDialog progress, Handler handler) {
		this.progress=progress;
		this.handler = handler;

	}

	public static void clear() {
		imagesList = new ArrayList<Mat>();
		pointsList = new ArrayList<List<KeyPoint>>();
		goodPointsList = new ArrayList<ArrayList<KeyPoint>>();
		matchesMap = new HashMap<MyPair<Integer, Integer>, ArrayList<DMatch>>();
		pcloud = new ArrayList<CloudPoint>();
		pMats = new HashMap<Integer, Mat>();

		size = new Size();
	}

	public void start() {

		progress.setProgress(0);
		progress.setMessage("setting calibration matrix...");

		MyMatrix.setCalibMatrix(progress);

		progress.setMessage("detecting features...");
		MyFeatures mf = new MyFeatures(progress);
		mf.detectFeatures();

		progress.setMessage("matching features...");
		mf.matchFeatures();

		MyTriangulation mt = new MyTriangulation(progress);


		
		if (!mt.getBaseLineTriangulation()) {
			handler.sendEmptyMessage(State.FAILURE.ordinal());
			// progress.parent.finish();
			progress.dismiss();
			return;
		}
		
		progress.setMessage("filling baseline...");
		MyReconFill.fillRecon(progress);

		size = imagesList.get(0).size();

		saveFile();

		MyProjection mp = MyProjection.getInstance();
		MyProjection.getResults(handler);

	//	source = MyProjection.getSource();
		progress.dismiss();
		handler.sendEmptyMessage(State.RECONSTRUCTED.ordinal());
	}

	private void saveFile() {
		handler.post(new Runnable() {

			@Override
			public void run() {
				AlertDialog.Builder builder = new AlertDialog.Builder(
						progress.parent);
				builder.setTitle("save as csv?");

				InputFilter filter = new InputFilter() {
					@Override
					public CharSequence filter(CharSequence source, int start,
							int end, Spanned dest, int dstart, int dend) {
						for (int i = start; i < end; i++) {
							if (!Character.isLetterOrDigit(source.charAt(i))) {
								return "";
							}
							if (i > 8)
								return "";
						}
						return null;
					}

				};

				final EditText input = new EditText(progress.parent);
				input.setInputType(InputType.TYPE_CLASS_TEXT);
				input.setFilters(new InputFilter[] { filter });
				input.setText("cloud");
				builder.setView(input);

				builder.setPositiveButton("OK",
						new DialogInterface.OnClickListener() {
							@Override
							public void onClick(DialogInterface dialog,
									int which) {
								String filename = input.getText().toString();
								Tools.cloud2csv(filename);
							}
						});
				builder.setNegativeButton("Cancel",
						new DialogInterface.OnClickListener() {
							@Override
							public void onClick(DialogInterface dialog,
									int which) {
								dialog.cancel();
							}
						});

				builder.show();
				// TODO Auto-generated method stub

			}
		});

	}
}
