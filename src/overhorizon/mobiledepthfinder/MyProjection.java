package overhorizon.mobiledepthfinder;

import java.util.ArrayList;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;

import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.os.Handler;

public class MyProjection {

	public static Double xAngle = 0., yAngle = 0., zAngle = 0.;
	public static Double xPos = 0., yPos = 0., zPos = 0.;
	public static Bitmap resultImage;

	static double xMean, xMin, xMax;
	static double yMean, yMin, yMax;
	static double zMean, zMin, zMax;
	static ArrayList<Point3> src;

	private static MyProjection instance = null;

	// only way to instantiate this singleton class object
	static MyProjection getInstance() {
		if (instance == null)
			return new MyProjection();
		return instance;

	}

	@Override
	protected void finalize() throws Throwable {
		instance = null;
		super.finalize();
	}

	private MyProjection() {
		xAngle = yAngle = zAngle = 0.;
		xPos = yPos = zPos = 0.;

		resultImage = Bitmap.createBitmap((int) Box.size.width,
				(int) Box.size.height, Config.ARGB_8888);

		src = Tools.cloudPoints2Points3(Box.pcloud);

		// -----------------------cube generated for testing - 
		// -----------------------comment when not needed

	/*	src = new ArrayList<Point3>();
		src.add(new Point3(-15, -15, -15));
		src.add(new Point3(3, -15, -15));
		src.add(new Point3(-15, 3, -15));
		src.add(new Point3(3, 3, -15));
		src.add(new Point3(-15, -15, 3));
		src.add(new Point3(3, -15, 3));
		src.add(new Point3(-15, 3, 3));
		src.add(new Point3(3, 3, 3));*/
		// ---------------------------------

		xMin = xMax = src.get(0).x;
		yMin = yMax = src.get(0).y;
		zMin = zMax = src.get(0).z;

		for (Point3 p : src) {
			if (p.x < xMin)
				xMin = p.x;
			if (p.x > xMax)
				xMax = p.x;
			if (p.y < yMin)
				yMin = p.y;
			if (p.y > yMax)
				yMax = p.y;
			if (p.z < zMin)
				zMin = p.z;
			if (p.z > zMax)
				zMax = p.z;

		}

		xMean = (xMin + xMax) / (double) 2.;
		yMean = (yMin + yMax) / (double) 2.;
		zMean = (zMin + zMax) / (double) 2.;

		for (Point3 p : src) {
			p.x -= xMean;
			p.y -= yMean;
			p.z -= zMean;
		}

		// ------------ offset calculation

		{
			double a = (Math.abs(xMin) + Math.abs(xMax))/2;
			double b = (Math.abs(yMin) + Math.abs(yMax))/2;
			double c = (Math.abs(zMin) + Math.abs(zMax))/2;
			double offset = a > b && a > c ? a : (b > c ? b : c);

			zMean = offset+1;
		}
	
		// -----------------------

		
	}

	static void getResults(Handler handler) {
		handler.removeMessages(Box.State.UPDATE_VIEW.ordinal());

		ArrayList<Point3> srcTemp = new ArrayList<Point3>();
		for (Point3 p : src) {
			srcTemp.add(p.clone());
		}

		Mat R = findRMat();
		for (Point3 p : srcTemp) {

			Mat point = Mat.ones(4, 1, CvType.CV_64FC1);
			point.put(0, 0, p.x);
			point.put(1, 0, p.y);
			point.put(2, 0, p.z);
			point.put(3, 0, 1);

			Mat point1 = Tools.myMulMat(R, point);

			p.x = point1.get(0, 0)[0];
			p.y = point1.get(1, 0)[0];
			p.z = point1.get(2, 0)[0];

		}
			
		Mat image;
		while((image = generateImage(srcTemp))==null){
			zPos++;
		}
		
		Utils.matToBitmap(image, resultImage, true);
		handler.sendEmptyMessage(Box.State.UPDATE_VIEW.ordinal());

	}
	

	private static Mat generateImage(ArrayList<Point3> srcTemp){
		for (Point3 p : srcTemp) {
			p.x += xPos;
			p.y += yPos;
			p.z += (zMean + zPos);
		}

		MatOfPoint3f srcMat = new MatOfPoint3f();
		srcMat.fromList(srcTemp);

		Mat tvec = Mat.zeros(3, 1, CvType.CV_64FC1);

		Mat rmat = Mat.eye(3, 3, CvType.CV_64FC1);
		Mat rvec = new Mat(3, 1, CvType.CV_64FC1);
		Calib3d.Rodrigues(rmat, rvec);

		MatOfDouble distortMat = new MatOfDouble(Box.distortionMat);
		MatOfPoint2f dest = new MatOfPoint2f();

		//----setting higher focal values reduces lensing
		
		Box.intrinsicMat.put(0, 0, 100);
		Box.intrinsicMat.put(1, 1, 100);
		
		//-----------------------------------------------
		
		Calib3d.projectPoints(srcMat, rvec, tvec, Box.intrinsicMat, distortMat,
				dest);

		ArrayList<Point> destArray = new ArrayList<Point>(dest.toList());

		Mat image = Mat.zeros((int) Box.size.height, (int) Box.size.width,
				CvType.CV_8UC1);

		Rect viewRect = new Rect(0, 0, (int) Box.size.width,
				(int) Box.size.height);
		for (int i = 0; i < destArray.size(); i++) {
			Point pDest = destArray.get(i);
			if (pDest.inside(viewRect)) {
				image.put((int) pDest.y, (int) pDest.x, 255);
			}
			else {
				for (Point3 p : srcTemp) {
					p.x -= xPos; 
					p.y -= yPos; 
					p.z -= (zMean + zPos);
				}

				return null;
			}
			
		}
		return image;
		
	}
	
	
	
	private static Mat findRMat() {

		Mat xMat = Mat.zeros(4, 4, CvType.CV_64FC1);
		xMat.put(0, 0, 1);
		xMat.put(3, 3, 1);

		xMat.put(1, 1, Math.cos(Math.toRadians(xAngle)));
		xMat.put(1, 2, -1 * Math.sin(Math.toRadians(xAngle)));

		xMat.put(2, 1, Math.sin(Math.toRadians(xAngle)));
		xMat.put(2, 2, Math.cos(Math.toRadians(xAngle)));
		// --------------------------------------
		Mat yMat = Mat.zeros(4, 4, CvType.CV_64FC1);
		yMat.put(1, 1, 1);
		yMat.put(3, 3, 1);

		yMat.put(0, 0, Math.cos(Math.toRadians(yAngle)));
		yMat.put(0, 2, Math.sin(Math.toRadians(yAngle)));

		yMat.put(2, 0, -1 * Math.sin(Math.toRadians(yAngle)));
		yMat.put(2, 2, Math.cos(Math.toRadians(yAngle)));
		// --------------------------------------
		Mat zMat = Mat.zeros(4, 4, CvType.CV_64FC1);
		zMat.put(2, 2, 1);
		zMat.put(3, 3, 1);

		zMat.put(0, 0, Math.cos(Math.toRadians(zAngle)));
		zMat.put(0, 1, -1 * Math.sin(Math.toRadians(zAngle)));

		zMat.put(1, 0, Math.sin(Math.toRadians(zAngle)));
		zMat.put(1, 1, Math.cos(Math.toRadians(zAngle)));
		// --------------------------------------
		Mat mulXY = Tools.myMulMat(xMat, yMat);
		Mat mulXYZ = Tools.myMulMat(mulXY, zMat);
		return mulXYZ;

	}

}
