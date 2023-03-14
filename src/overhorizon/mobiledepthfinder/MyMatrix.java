package overhorizon.mobiledepthfinder;

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.KeyPoint;
import org.opencv.utils.Converters;

import android.util.DisplayMetrics;
import android.util.Log;

public class MyMatrix {
	static void getAlignedPointsFromMatch(List<KeyPoint> list,
			List<KeyPoint> list2, ArrayList<DMatch> matches,
			List<KeyPoint> imgpts1_tmp, List<KeyPoint> imgpts2_tmp) {

		for (int i = 0; i < matches.size() ; i++) {
	//		if (matches.get(i).queryIdx < list.size()
	//				&& matches.get(i).trainIdx < list2.size()) 
			{
				imgpts1_tmp.add(list.get(matches.get(i).queryIdx));
				imgpts2_tmp.add(list2.get(matches.get(i).trainIdx));
			}

		}

	}

	static void setCalibMatrix(MyProgressDialog progress) {
		
		int w = Box.imagesList.get(0).width();
		int h = Box.imagesList.get(0).height();
		
		DisplayMetrics dm = new DisplayMetrics();
		progress.parent.getWindowManager().getDefaultDisplay().getMetrics(dm);

					
		float xDpi = dm.xdpi;
		float yDpi = dm.ydpi;
		
	
				
		double focalx=xDpi*Box.focal/25.4;
		double focaly=yDpi*Box.focal/25.4;
			
		Box.distortionMat = Mat.zeros(1, 4, CvType.CV_64FC1);

		Box.intrinsicMat = Mat.zeros(3, 3, CvType.CV_64FC1);
		Box.intrinsicMat.put(0, 0,/* Math.max(w,h)*/focalx);
		Box.intrinsicMat.put(0, 2, w / 2.0);
		Box.intrinsicMat.put(1, 1, /*Math.max(w,h)*/focaly);
		Box.intrinsicMat.put(1, 2, h / 2.0);
		Box.intrinsicMat.put(2, 2, 1);
		Log.i(Box.TAG, "focal lenght is " + Box.focal);

		Box.invIntrinsicMat = new Mat(3, 3, CvType.CV_64FC1);
		Core.invert(Box.intrinsicMat, Box.invIntrinsicMat);
		progress.setProgress(99);
	}

	static MultiValue<ArrayList<DMatch>, Mat, Mat, Mat> getFundamentalMat(
			List<KeyPoint> list, List<KeyPoint> list2,
			ArrayList<KeyPoint> imgpts1_good, ArrayList<KeyPoint> imgpts2_good,
			ArrayList<DMatch> matches) {

		// Try to eliminate keypoints based on the fundamental matrix
		// (although this is not the proper way to do this)

	
		imgpts1_good.clear();
		imgpts2_good.clear();

		List<KeyPoint> imgpts1_tmp = new ArrayList<KeyPoint>();
		List<KeyPoint> imgpts2_tmp = new ArrayList<KeyPoint>();
		if (matches.size() <= 0) {

			Log.i(Box.TAG, "getFundamentalMat- no points aligned");
			return null;

		} else {
			getAlignedPointsFromMatch(list, list2, matches, imgpts1_tmp,
					imgpts2_tmp);
		}

		//List<Point> pts1 = Tools.keyPoints2Points(imgpts1_tmp);
		//List<Point> pts2 = Tools.keyPoints2Points(imgpts2_tmp);
		ArrayList<MyPoint> pts1 = Tools.keyPoints2MyPoints(imgpts1_tmp);	
		ArrayList<MyPoint> pts2 = Tools.keyPoints2MyPoints(imgpts2_tmp);
		
		
		
		
		MatOfPoint2f mpts1 = new MatOfPoint2f(
				(Point[]) pts1.toArray(new Point[0]));
		MatOfPoint2f mpts2 = new MatOfPoint2f(
				(Point[]) pts2.toArray(new Point[0]));
		
		double maxVal=Tools.findMax(pts1).getValue();
		
		/*MatOfPoint2f mpts1 = new MatOfPoint2f();
		mpts1.fromList(pts1);
		MatOfPoint2f mpts2 = new MatOfPoint2f();
		mpts2.fromList(pts2);*/
		
		Mat status = new Mat();

		Mat F = Calib3d.findFundamentalMat(mpts1, mpts2, Calib3d.FM_RANSAC,
				0.006 * (maxVal), 0.99, status); 

		ArrayList<DMatch> new_matches = new ArrayList<DMatch>();
		
		ArrayList<Byte> statusList = new ArrayList<Byte>();
		Converters.Mat_to_vector_uchar(status, statusList);

		for (int i = 0; i < statusList.size(); i++) {
			if (statusList.get(i) != 0) {
				imgpts1_good.add(imgpts1_tmp.get(i));
				imgpts2_good.add(imgpts2_tmp.get(i));

				if (matches.size() <= 0) {
					new_matches.add(new DMatch(matches.get(i).queryIdx,matches.get(i).trainIdx,matches.get(i).distance));
				} else {
								
				new_matches.add(matches.get(i));
				
				}
			}
		}

		return new MultiValue<ArrayList<DMatch>, Mat, Mat, Mat>(true,
				new_matches, F);
	}

	// -------------------------------------------------------
	// -------------------------------------------------------

	static Boolean findCameraMatrices(Mat K, Mat Kinv, Mat distcoeff,
			List<KeyPoint> list, List<KeyPoint> list2,
			ArrayList<KeyPoint> imgpts1_good, ArrayList<KeyPoint> imgpts2_good,
			Mat P, Mat P1, ArrayList<DMatch> matches,
			ArrayList<CloudPoint> outCloud) {
		{

			MultiValue<ArrayList<DMatch>, Mat, Mat, Mat> mv = getFundamentalMat(
					list, list2, imgpts1_good, imgpts2_good, matches);
			if (mv==null) return false;
			matches = mv.getFirst();
			Mat F = mv.getSecond();

			
			 if (matches.size() <50){// 100)  ||
	//		  ((double)imgpts1_good.size() // 
	//				  (double)imgpts1.size()) < 0.25
			  Log.i(Box.TAG, "not enough matches"); 
			  return false; 
			  }
			 

			// Calculate Essential

			Mat E = new Mat();
			E = Tools.myMulMat(K.t(), F);
			E = Tools.myMulMat(E, K);

			// check the Essential

			if (Math.abs(Core.determinant(E)) > 1e-07) {

				Log.i(Box.TAG,
						"Essential Matrix determinant: " + Core.determinant(E));

				P1 = Mat.zeros(3, 4, CvType.CV_64FC1);
				return false;
			}

			Mat R1 = Mat.zeros(3, 3, CvType.CV_64FC1);
			Mat R2 = Mat.zeros(3, 3, CvType.CV_64FC1);
			Mat t1 = Mat.zeros(1, 3, CvType.CV_64FC1);
			Mat t2 = Mat.zeros(1, 3, CvType.CV_64FC1);
			MultiValue<Mat, Mat, Mat, Mat> result = new MultiValue<Mat, Mat, Mat, Mat>(
					false);

			// decompose E to P' , HZ (9.19)
			{
				result = DecomposeEtoRandT(E, R1, R2, t1, t2);
				R1 = result.getFirst();
				if (!result.isTrue())
					return false;

				if (Core.determinant(R1) + 1.0 < 1e-09) {

					E = E.mul(new Mat(3, 3, CvType.CV_64FC1, new Scalar(-1.0)));
					result = DecomposeEtoRandT(E, R1, R2, t1, t2);
					R1 = result.getFirst();
				}

				R2 = result.getSecond();
				t1 = result.getThird();
				t2 = result.getFourth();

				if (!CheckCoherentRotation(R1)) {

					P1 = Mat.zeros(3, 4, CvType.CV_64FC1);
					return false;
				}

				P1.put(0, 0, R1.get(0, 0)[0]);
				P1.put(0, 1, R1.get(0, 1)[0]);
				P1.put(0, 2, R1.get(0, 2)[0]);
				P1.put(0, 3, t1.get(0, 0)[0]);
				P1.put(1, 0, R1.get(1, 0)[0]);
				P1.put(1, 1, R1.get(1, 1)[0]);
				P1.put(1, 2, R1.get(1, 2)[0]);
				P1.put(1, 3, t1.get(1, 0)[0]);
				P1.put(2, 0, R1.get(2, 0)[0]);
				P1.put(2, 1, R1.get(2, 1)[0]);
				P1.put(2, 2, R1.get(2, 2)[0]);
				P1.put(2, 3, t1.get(2, 0)[0]);

				Log.i(Box.TAG, "Test for P1");

				ArrayList<CloudPoint> pcloud = new ArrayList<CloudPoint>(), pcloud1 = new ArrayList<CloudPoint>();
				ArrayList<KeyPoint> corresp = new ArrayList<KeyPoint>();

				double reproj_error1 = MyTriangulation.triangulatePoints(
						imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1,
						pcloud, corresp);
				double reproj_error2 = MyTriangulation.triangulatePoints(
						imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P,
						pcloud1, corresp);
				ArrayList<Byte> status = new ArrayList<Byte>();
				// check if pointa are triangulated --in front-- of cameras for
				// all 4 ambiguations
				if (!MyTriangulation.testTriangulation(pcloud, P1, status)
						|| !MyTriangulation.testTriangulation(pcloud1, P,
								status) || reproj_error1 > 100.0
						|| reproj_error2 > 100.0) {
					P1.put(0, 0, R1.get(0, 0)[0]);
					P1.put(0, 1, R1.get(0, 1)[0]);
					P1.put(0, 2, R1.get(0, 2)[0]);
					P1.put(0, 3, t2.get(0, 0)[0]);
					P1.put(1, 0, R1.get(1, 0)[0]);
					P1.put(1, 1, R1.get(1, 1)[0]);
					P1.put(1, 2, R1.get(1, 2)[0]);
					P1.put(1, 3, t2.get(1, 0)[0]);
					P1.put(2, 0, R1.get(2, 0)[0]);
					P1.put(2, 1, R1.get(2, 1)[0]);
					P1.put(2, 2, R1.get(2, 2)[0]);
					P1.put(2, 3, t2.get(2, 0)[0]);

					Log.i(Box.TAG, "Test for P1");

					pcloud.clear();
					pcloud1.clear();
					corresp = new ArrayList<KeyPoint>();
					reproj_error1 = MyTriangulation.triangulatePoints(
							imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P,
							P1, pcloud, corresp);
					reproj_error2 = MyTriangulation.triangulatePoints(
							imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1,
							P, pcloud1, corresp);

					if (!MyTriangulation.testTriangulation(pcloud, P1, status)
							|| !MyTriangulation.testTriangulation(pcloud1, P,
									status) || reproj_error1 > 100.0
							|| reproj_error2 > 100.0) {
						if (!CheckCoherentRotation(R2)) {

							Log.i(Box.TAG, "bad rotation");
							P1 = Mat.zeros(4, 3, CvType.CV_64FC1);
							return false;
						}
						P1.put(0, 0, R2.get(0, 0)[0]);
						P1.put(0, 1, R2.get(0, 1)[0]);
						P1.put(0, 2, R2.get(0, 2)[0]);
						P1.put(0, 3, t1.get(0, 0)[0]);
						P1.put(1, 0, R2.get(1, 0)[0]);
						P1.put(1, 1, R2.get(1, 1)[0]);
						P1.put(1, 2, R2.get(1, 2)[0]);
						P1.put(1, 3, t1.get(1, 0)[0]);
						P1.put(2, 0, R2.get(2, 0)[0]);
						P1.put(2, 1, R2.get(2, 1)[0]);
						P1.put(2, 2, R2.get(2, 2)[0]);
						P1.put(2, 3, t1.get(2, 0)[0]);

						Log.i(Box.TAG, "Test for P1");

						pcloud.clear();
						pcloud1.clear();
						corresp = new ArrayList<KeyPoint>();
						reproj_error1 = MyTriangulation.triangulatePoints(
								imgpts1_good, imgpts2_good, K, Kinv, distcoeff,
								P, P1, pcloud, corresp);
						reproj_error2 = MyTriangulation.triangulatePoints(
								imgpts2_good, imgpts1_good, K, Kinv, distcoeff,
								P1, P, pcloud1, corresp);

						if (!MyTriangulation.testTriangulation(pcloud, P1,
								status)
								|| !MyTriangulation.testTriangulation(pcloud1,
										P, status)
								|| reproj_error1 > 100.0
								|| reproj_error2 > 100.0) {
							P1.put(0, 0, R2.get(0, 0)[0]);
							P1.put(0, 1, R2.get(0, 1)[0]);
							P1.put(0, 2, R2.get(0, 2)[0]);
							P1.put(0, 3, t2.get(0, 0)[0]);
							P1.put(1, 0, R2.get(1, 0)[0]);
							P1.put(1, 1, R2.get(1, 1)[0]);
							P1.put(1, 2, R2.get(1, 2)[0]);
							P1.put(1, 3, t2.get(1, 0)[0]);
							P1.put(2, 0, R2.get(2, 0)[0]);
							P1.put(2, 1, R2.get(2, 1)[0]);
							P1.put(2, 2, R2.get(2, 2)[0]);
							P1.put(2, 3, t2.get(2, 0)[0]);

							Log.i(Box.TAG, "Test for P1");

							pcloud.clear();
							pcloud1.clear();
							corresp = new ArrayList<KeyPoint>();
							reproj_error1 = MyTriangulation.triangulatePoints(
									imgpts1_good, imgpts2_good, K, Kinv,
									distcoeff, P, P1, pcloud, corresp);
							reproj_error2 = MyTriangulation.triangulatePoints(
									imgpts2_good, imgpts1_good, K, Kinv,
									distcoeff, P1, P, pcloud1, corresp);

							if (!MyTriangulation.testTriangulation(pcloud, P1,
									status)
									|| !MyTriangulation.testTriangulation(
											pcloud1, P, status)
									|| reproj_error1 > 100.0
									|| reproj_error2 > 100.0) {
								// commented for tests Log.e(Box.TAG,
								// "havent foun good P1");
								return false;
							}
						}
					}
				}
				for (int i = 0; i < pcloud.size(); i++) {
					outCloud.add(pcloud.get(i));
				}
			}

		}
		return true;
	}

	// -------------------------------------------------------
	// -------------------------------------------------------

	static boolean CheckCoherentRotation(Mat r1) {

		if (Math.abs(Core.determinant(r1)) - 1.0 > 1e-07) {
			Log.e(Box.TAG, "this is not a rotation matrix");
			return false;
		}

		return true;

	}

	// -------------------------------------------------------
	// -------------------------------------------------------

	private static MultiValue<Mat, Mat, Mat, Mat> DecomposeEtoRandT(Mat E,
			Mat R1, Mat R2, Mat t1, Mat t2) {
		// #ifdef DECOMPOSE_SVD
		// Using HZ E decomposition

		Mat svd_u = new Mat(), svd_vt = new Mat(), svd_w = new Mat();
		Core.SVDecomp(E, svd_w, svd_u, svd_vt, Core.SVD_MODIFY_A);

		// check if first and second singular values are the same (as they
		// should be)
		Log.i(Box.TAG,
				"svd values=" + svd_w.get(0, 0)[0] + "::" + svd_w.get(1, 0)[0]
						+ "::" + svd_w.get(2, 0)[0]);
		
		double singular_values_ratio = Math.abs(svd_w.get(0, 0)[0]
				/ svd_w.get(1, 0)[0]);

		if (singular_values_ratio > 1.0)
			singular_values_ratio = 1.0 / singular_values_ratio; // flip ratio
																	// to keep
																	// it [0,1]
		if (singular_values_ratio < 0.3) {
			// testy tylko
			Log.e(Box.TAG, "too wide singulars");
			return new MultiValue<Mat, Mat, Mat, Mat>(false);

		}

		Mat W = Mat.zeros(3, 3, CvType.CV_64FC1);
		W.put(0, 1, -1);
		W.put(1, 0, 1);
		W.put(2, 2, 1);

		Mat Wt = Mat.zeros(3, 3, CvType.CV_64FC1);
		Wt.put(0, 1, 1);
		Wt.put(1, 0, -1);
		Wt.put(2, 2, 1);

		R1 = Tools.myMulMat(svd_u, W);
		R1 = Tools.myMulMat(R1, svd_vt);

		R2 = Tools.myMulMat(svd_u, Wt);
		R2 = Tools.myMulMat(R2, svd_vt);

		t1 = svd_u.col(2); // u3
		t2 = t1.clone();
		for (int i = 0; i < t2.rows(); i++) {
			t2.put(i, 0, t2.get(i, 0)[0] * -1);
		}

		return new MultiValue<Mat, Mat, Mat, Mat>(true, R1, R2, t1, t2);
	}
	// -------------------------------------------------------
	// -------------------------------------------------------

}
