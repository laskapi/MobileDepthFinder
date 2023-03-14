package overhorizon.mobiledepthfinder;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.features2d.KeyPoint;

import android.util.Log;

public class Tools {

	static <T extends Comparable<T>> T findMin(ArrayList<T> list) {
		T result = list.get(0);
		for (T t : list) {

			if (t.compareTo(result) < 0) {
				result = t;
			}
		}
		return result;
	}

	static <T extends Comparable<T>> T findMax(ArrayList<T> list) {
		T result = list.get(0);
		for (T t : list) {

			if (t.compareTo(result) > 0) {
				result = t;
			}
		}
		return result;
	}

	static ArrayList<MyPoint> matOfKeyPoints2ListofMyPoints(MatOfKeyPoint source) {
		ArrayList<KeyPoint> temp = (ArrayList<KeyPoint>) source.toList();
		ArrayList<MyPoint> result = new ArrayList<MyPoint>();
		for (KeyPoint k : temp) {
			result.add((MyPoint) k.pt);
		}
		return result;
	}

	
	
	static MatOfPoint2f listOfMyPoints2MatOfPoints(List<MyPoint> source) {
		/*
		 * List<Point> temp = new ArrayList<Point>(); for (MyPoint m : source) {
		 * temp.add(m); } MatOfPoint2f result = new MatOfPoint2f();
		 * result.fromList(source);
		 */
		List<Point> temp = new ArrayList<Point>(source);
		MatOfPoint2f result = new MatOfPoint2f();
		result.fromList(temp);
		return result;

	}

	static ArrayList<MyPoint> keyPoints2MyPoints(List<KeyPoint> imgpts1_tmp) {
		ArrayList<MyPoint> result = new ArrayList<MyPoint>();
		for (KeyPoint k : imgpts1_tmp) {
			result.add(new MyPoint(k.pt));
		}
		return result;
	}

	static List<Point> keyPoints2Points(List<KeyPoint> imgpts1_tmp) {
		List<Point> result = new ArrayList<Point>();
		for (KeyPoint k : imgpts1_tmp) {
			result.add(k.pt.clone());
		}
		return result;
	}

	
	
	@SuppressWarnings("unchecked")
	static ArrayList<MyPoint> point2MyPoint(ArrayList<Point> source) {
		ArrayList<? extends Point> result = source;
		return (ArrayList<MyPoint>) result;
	}

	static Mat myMulMat(Mat first, Mat second) {
		Mat result = new Mat(first.rows(), second.cols(), first.type());
		Core.gemm(first, second, 1, new Mat(), 0, result, 0);
		return result;

	}

	static ArrayList<Point3>cloudPoints2Points3(ArrayList<CloudPoint> cpts) {
		ArrayList<Point3> out = new ArrayList<Point3>();
		for (int i = 0; i < cpts.size(); i++) {
			out.add(cpts.get(i).pt.clone());
		}
		return out;
	}

	static Boolean cloud2csv(String filename) {
		
		
		
		
		
		try {
			String resultString = "xcoord,ycoord,zcoord\n";

			for (int i = 0; i < Box.pcloud.size(); i++) {

				Point3 p = Box.pcloud.get(i).pt;
				resultString += Double.valueOf(p.x).toString();
				resultString += ",";
				resultString += Double.valueOf(p.y).toString();
				resultString += ",";
				resultString += Double.valueOf(p.z).toString();
				resultString += "\n";
			}
			Log.i("the string result is", resultString);

			File myFile = new File("/sdcard/depthfinder/"+filename+".csv");
			myFile.createNewFile();
			FileOutputStream fOut = new FileOutputStream(myFile);
			OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);
			myOutWriter.append(resultString);
			myOutWriter.close();
			fOut.close();

		} catch (IOException ioe) {
			ioe.printStackTrace();
		}

		return true;
	}
}