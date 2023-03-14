package overhorizon.mobiledepthfinder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.KeyPoint;
import org.opencv.utils.Converters;

import android.util.Log;

public class MyTriangulation {
	MyProgressDialog progress;

	public MyTriangulation(MyProgressDialog progress) {
		this.progress = progress;
	}

	Boolean getBaseLineTriangulation() {
		progress.setMessage("matches sort");
		progress.setProgress(0);
		Log.i(Box.TAG, "triangulating base");

		Mat P = Mat.zeros(3, 4, CvType.CV_64FC1);
		P.put(0, 0, 1);
		P.put(1, 1, 1);
		P.put(2, 2, 1);
		Mat P1 = P.clone();

		ArrayList<CloudPoint> tmp_pcloud = new ArrayList<CloudPoint>();

		// sort pairwise matches to find the lowest Homography inliers
		// [Snavely07 4.2]

		Log.i("depthFinder", "Find highest match...");
		ArrayList<MyPair<Integer, MyPair<Integer, Integer>>> matches_sizes = new ArrayList<MyPair<Integer, MyPair<Integer, Integer>>>();

		int progressCount=0;
		
		for (Map.Entry<MyPair<Integer, Integer>, ArrayList<DMatch>> entry : Box.matchesMap
				.entrySet()) {
			
			progress.setProgress(((double) progressCount)
					/ ((double) Box.matchesMap.size()));
			
			
			
			Log.i(Box.TAG, "next map entry");
			MyPair<Integer, MyPair<Integer, Integer>> tempPair = new MyPair<Integer, MyPair<Integer, Integer>>(
					0, new MyPair<Integer, Integer>(0, 0));
			tempPair.second = (MyPair<Integer, Integer>) entry.getKey();
			if (entry.getValue().size() < 100) {
				tempPair.first = 100;

				matches_sizes.add(tempPair);
			} else {
				int Hinliers = findHomographyInliers2Views(
						entry.getKey().first, entry.getKey().second);
				int percent = (int) (((double) Hinliers)
						/ ((double) entry.getValue().size()) * 100.0);
				tempPair.first = percent;
				matches_sizes.add(tempPair);

			}
			progressCount++;
		}
		Collections.sort(matches_sizes);

		// Reconstruct from two views
		Boolean goodF = false;
//		int highest_pair = 0;
		Box.m_first_view = 0;
		Box.m_second_view = 0;
		// reverse iterate by number of matches

		// ---------------uwaga testy
		
		progress.setMessage("baseline triangulation...");
		progress.setProgress(0);
		progressCount=0;
		for (MyPair<Integer, MyPair<Integer, Integer>> entry : matches_sizes) {
			
			// for (Map.Entry<MyPair<Integer, Integer>, ArrayList<DMatch>> entry
			// : Box.matchesMap .entrySet()) {
			// progression++;
			progress.setProgress(((double) progressCount)
					/ ((double) Box.matchesMap.size()));
			// for (Box.m_first_view = 0; Box.m_first_view <
			// Box.imagesList.size() - 1; Box.m_first_view++) {
			// for (Box.m_second_view = Box.m_first_view + 1; Box.m_second_view
			// < Box.imagesList
			// .size(); Box.m_second_view++) {
			// ArrayList<DMatch> entry = Box.matchesMap
			// .get(new MyPair<Integer, Integer>(Box.m_first_view,
			// Box.m_second_view));

			if (goodF)
				break;

			// Box.m_first_view = entry.getKey().first;
			// Box.m_second_view = entry.getKey().second;

			Box.m_first_view = entry.second.first;
			Box.m_second_view = entry.second.second;
			// --------
			for (int i = 0; i < Box.imagesList.size(); i++) {
				Box.goodPointsList.add(new ArrayList<KeyPoint>());
			}

			// ---------

			Log.i(Box.TAG, "calculate Matrixes for image : " + Box.m_first_view
					+ " & " + Box.m_second_view);

			goodF = MyMatrix.findCameraMatrices(Box.intrinsicMat,
					Box.invIntrinsicMat, Box.distortionMat, Box.pointsList
							.get(Box.m_first_view), Box.pointsList
							.get(Box.m_second_view), Box.goodPointsList
							.get(Box.m_first_view), Box.goodPointsList
							.get(Box.m_second_view), P, P1, Box.matchesMap
							.get(new MyPair<Integer, Integer>(Box.m_first_view,
									Box.m_second_view)), tmp_pcloud);

			if (goodF) {
				ArrayList<CloudPoint> new_triangulated = new ArrayList<CloudPoint>();
				ArrayList<Integer> add_to_cloud = new ArrayList<Integer>();

				Box.pMats.put(Box.m_first_view, P);
				Box.pMats.put(Box.m_second_view, P1);

				Boolean good_triangulation = triangulatePointsBetweenViews(
						Box.m_second_view, Box.m_first_view, new_triangulated,
						add_to_cloud);

				if (!good_triangulation
						|| Collections.frequency(add_to_cloud,
								Integer.valueOf((int) 1)) < 10) {

					goodF = false;

					Box.pMats.put(Box.m_first_view,
							Mat.zeros(3, 4, CvType.CV_64FC1));
					Box.pMats.put(Box.m_second_view,
							Mat.zeros(3, 4, CvType.CV_64FC1));
					Box.m_second_view++;
				} else {

					for (int j = 0; j < add_to_cloud.size(); j++) {
						if (add_to_cloud.get(j) == 1) {
							Box.pcloud.add(new_triangulated.get(j));
						}
					}

				}
			}
			progressCount++;
		}
		progress.setProgress(99);
		if (!goodF) {
			Log.e(Box.TAG, "cant create base");
			return false;
		} else {
			Log.i(Box.TAG, "base created from view " + Box.m_first_view
					+ " and " + Box.m_second_view);
			return true;
			// Box.pcloud.addAll(tmp_pcloud);
		}
	}

	private int findHomographyInliers2Views(Integer i, Integer j) {

		ArrayList<KeyPoint> ikpts = new ArrayList<KeyPoint>(), jkpts = new ArrayList<KeyPoint>();
		ArrayList<MyPoint> ipts, jpts;
		MyMatrix.getAlignedPointsFromMatch(Box.pointsList.get(i),
				Box.pointsList.get(j),
				Box.matchesMap.get(new MyPair<Integer, Integer>(i, j)), ikpts,
				jkpts);
		ipts = Tools.keyPoints2MyPoints(ikpts);
		jpts = Tools.keyPoints2MyPoints(jkpts);

		MyPoint maxVal = Tools.findMax(jpts); 

		Mat status = new Mat();

		Mat H = Calib3d.findHomography(Tools.listOfMyPoints2MatOfPoints(ipts),
				Tools.listOfMyPoints2MatOfPoints(jpts), Calib3d.RANSAC,
				0.004 * (maxVal.getValue()), status); // threshold from //
														// Snavely07
		ArrayList<Double> result = new ArrayList<Double>();
		for (int k = 0; k < status.rows(); k++) {
			for (int l = 0; l < status.cols(); l++) {
				result.add(status.get(k, l)[0]);
			}
		}

		return (result.size() - Collections.frequency(result,
				Double.valueOf(0.0))); // number of inliers

	}

	public static double triangulatePoints(ArrayList<KeyPoint> pt_set1,
			ArrayList<KeyPoint> pt_set2, Mat K, Mat Kinv, Mat distcoeff, Mat P,
			Mat P1, ArrayList<CloudPoint> pointcloud,
			ArrayList<KeyPoint> correspImg1Pt) {

		// pointcloud.clear();
		correspImg1Pt = new ArrayList<KeyPoint>();

		Mat P1_ = Mat.zeros(4, 4, CvType.CV_64FC1);

		P1_.put(0, 0, P1.get(0, 0)[0]);
		P1_.put(0, 1, P1.get(0, 1)[0]);
		P1_.put(0, 2, P1.get(0, 2)[0]);
		P1_.put(0, 3, P1.get(0, 3)[0]);
		P1_.put(1, 0, P1.get(1, 0)[0]);
		P1_.put(1, 1, P1.get(1, 1)[0]);
		P1_.put(1, 2, P1.get(1, 2)[0]);
		P1_.put(1, 3, P1.get(1, 3)[0]);
		P1_.put(2, 0, P1.get(2, 0)[0]);
		P1_.put(2, 1, P1.get(2, 1)[0]);
		P1_.put(2, 2, P1.get(2, 2)[0]);
		P1_.put(2, 3, P1.get(2, 3)[0]);
		P1_.put(3, 3, 1);

		Mat P1inv = new Mat(P1_.inv(), Range.all());

		// cout << "Triangulating...";

		ArrayList<Double> reproj_error = new ArrayList<Double>();
		int pts_size = pt_set1.size();

		/*
		 * //#if 0 //Using OpenCV's triangulation //convert to Point2f
		 * ArrayList<MyPoint> _pt_set1_pt=new
		 * ArrayList<MyPoint>(),_pt_set2_pt=new ArrayList<MyPoint>();
		 * _pt_set1_pt=Tools.keyPoints2MyPoints(pt_set1);
		 * _pt_set2_pt=Tools.keyPoints2MyPoints(pt_set2);
		 * 
		 * //undistort Mat pt_set1_pt,pt_set2_pt; undistortPoints(_pt_set1_pt,
		 * pt_set1_pt, K, distcoeff); undistortPoints(_pt_set2_pt, pt_set2_pt,
		 * K, distcoeff);
		 * 
		 * //triangulate Mat pt_set1_pt_2r = pt_set1_pt.reshape(1, 2); Mat
		 * pt_set2_pt_2r = pt_set2_pt.reshape(1, 2); Mat
		 * pt_3d_h(1,pts_size,CV_32FC4);
		 * cv::triangulatePoints(P,P1,pt_set1_pt_2r,pt_set2_pt_2r,pt_3d_h);
		 * 
		 * //calculate reprojection vector<Point3f> pt_3d;
		 * convertPointsHomogeneous(pt_3d_h.reshape(4, 1), pt_3d);
		 * cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P(0,0),P(0,1),P(0,2),
		 * P(1,0),P(1,1),P(1,2), P(2,0),P(2,1),P(2,2)); Vec3d rvec; Rodrigues(R
		 * ,rvec); Vec3d tvec(P(0,3),P(1,3),P(2,3)); vector<Point2f>
		 * reprojected_pt_set1;
		 * projectPoints(pt_3d,rvec,tvec,K,distcoeff,reprojected_pt_set1);
		 * 
		 * for (unsigned int i=0; i<pts_size; i++) { CloudPoint cp; cp.pt =
		 * pt_3d[i]; pointcloud.push_back(cp);
		 * reproj_error.push_back(norm(_pt_set1_pt[i]-reprojected_pt_set1[i]));
		 * } #else
		 */

		Mat KP1 = Tools.myMulMat(K, P1);

		for (int i = 0; i < pts_size; i++) {

			Point kp = pt_set1.get(i).pt;
			Point3 u = new Point3(kp.x, kp.y, 1.0);

			Mat utemp = new Mat(3, 1, CvType.CV_64FC1);
			utemp.put(0, 0, u.x);
			utemp.put(1, 0, u.y);
			utemp.put(2, 0, u.z);
			Mat um = Tools.myMulMat(Kinv, utemp);

			u.x = um.get(0, 0)[0];
			u.y = um.get(1, 0)[0];
			u.z = um.get(2, 0)[0];

			Point kp1 = pt_set2.get(i).pt;
			Point3 u1 = new Point3(kp1.x, kp1.y, 1.0);

			Mat utemp1 = new Mat(3, 1, CvType.CV_64FC1);
			utemp1.put(0, 0, u1.x);
			utemp1.put(1, 0, u1.y);
			utemp1.put(2, 0, u1.z);
			Mat um1 = Tools.myMulMat(Kinv, utemp1);

			u1.x = um1.get(0, 0)[0];
			u1.y = um1.get(1, 0)[0];
			u1.z = um1.get(2, 0)[0];

			Mat X = iterativeLinearLSTriangulation(u, P, u1, P1);

			Mat xPt_img = Tools.myMulMat(KP1, X); // reproject

			Point xPt_img_ = new Point(xPt_img.get(0, 0)[0]
					/ xPt_img.get(2, 0)[0], xPt_img.get(1, 0)[0]
					/ xPt_img.get(2, 0)[0]);

			Mat normat = new Mat(2, 1, CvType.CV_64FC1);
			normat.put(0, 0, xPt_img_.x - kp1.x);
			normat.put(1, 0, xPt_img_.y - kp1.y);
			Double reprj_err = Core.norm(normat);

			reproj_error.add(reprj_err);

			CloudPoint cp = new CloudPoint();
			cp.pt = new Point3(X.get(0, 0)[0], X.get(1, 0)[0], X.get(2, 0)[0]);
			cp.reprojection_error = reprj_err;

			pointcloud.add(cp);
			correspImg1Pt.add(pt_set1.get(i));

		}

		Scalar mse = Core.mean(Converters.vector_double_to_Mat(reproj_error));

		return mse.val[0];

	}

	private static Mat linearLSTriangulation(Point3 u, Mat P, Point3 u1, Mat P1) {

		Mat A = new Mat(4, 3, CvType.CV_64FC1);

		A.put(0, 0, (u.x * P.get(2, 0)[0] - P.get(0, 0)[0]));
		A.put(0, 1, (u.x * P.get(2, 1)[0] - P.get(0, 1)[0]));
		A.put(0, 2, (u.x * P.get(2, 2)[0] - P.get(0, 2)[0]));

		A.put(1, 0, (u.y * P.get(2, 0)[0] - P.get(1, 0)[0]));
		A.put(1, 1, (u.y * P.get(2, 1)[0] - P.get(1, 1)[0]));
		A.put(1, 2, (u.y * P.get(2, 2)[0] - P.get(1, 2)[0]));

		A.put(2, 0, (u1.x * P1.get(2, 0)[0] - P1.get(0, 0)[0]));
		A.put(2, 1, (u1.x * P1.get(2, 1)[0] - P1.get(0, 1)[0]));
		A.put(2, 2, (u1.x * P1.get(2, 2)[0] - P1.get(0, 2)[0]));

		A.put(3, 0, (u1.y * P1.get(2, 0)[0] - P1.get(1, 0)[0]));
		A.put(3, 1, (u1.y * P1.get(2, 1)[0] - P1.get(1, 1)[0]));
		A.put(3, 2, (u1.y * P1.get(2, 2)[0] - P1.get(1, 2)[0]));

		Mat B = new Mat(4, 1, CvType.CV_64FC1);

		B.put(0, 0, (u.x * P.get(2, 3)[0] - P.get(0, 3)[0]));
		B.put(1, 0, (u.y * P.get(2, 3)[0] - P.get(1, 3)[0]));
		B.put(2, 0, (u1.x * P1.get(2, 3)[0] - P1.get(0, 3)[0]));
		B.put(3, 0, (u1.y * P1.get(2, 3)[0] - P1.get(1, 3)[0]));

		Mat X = new Mat();
		Core.solve(A, B, X, Core.DECOMP_SVD);

		return X;
	}

	static Mat iterativeLinearLSTriangulation(Point3 u, Mat P, Point3 u1, Mat P1) {

		double wi = 1, wi1 = 1;
		Mat X = Mat.zeros(4, 1, CvType.CV_64FC1);
		for (int i = 0; i < 10; i++) {
			Mat X_ = linearLSTriangulation(u, P, u1, P1);

			X.put(0, 0, (X_.get(0, 0)[0]));
			X.put(1, 0, (X_.get(1, 0)[0]));
			X.put(2, 0, (X_.get(2, 0)[0]));
			X.put(3, 0, Double.valueOf(1.0));

			// recalculate weights

			double p2x = (Tools.myMulMat(P.row(2), X)).get(0, 0)[0];
			double p2x1 = (Tools.myMulMat(P1.row(2), X)).get(0, 0)[0];

			// breaking point
			if (Math.abs(wi - p2x) <= 0.0001 && Math.abs(wi1 - p2x1) <= 0.0001)
				break;

			wi = p2x;
			wi1 = p2x1;

			// reweight equations and solve
			Mat A = new Mat(4, 3, CvType.CV_64FC1);

			A.put(0, 0, (u.x * P.get(2, 0)[0] - P.get(0, 0)[0]) / wi);
			A.put(0, 1, (u.x * P.get(2, 1)[0] - P.get(0, 1)[0]) / wi);
			A.put(0, 2, (u.x * P.get(2, 2)[0] - P.get(0, 2)[0]) / wi);

			A.put(1, 0, (u.y * P.get(2, 0)[0] - P.get(1, 0)[0]) / wi);
			A.put(1, 1, (u.y * P.get(2, 1)[0] - P.get(1, 1)[0]) / wi);
			A.put(1, 2, (u.y * P.get(2, 2)[0] - P.get(1, 2)[0]) / wi);

			A.put(2, 0, (u1.x * P1.get(2, 0)[0] - P1.get(0, 0)[0]) / wi1);
			A.put(2, 1, (u1.x * P1.get(2, 1)[0] - P1.get(0, 1)[0]) / wi1);
			A.put(2, 2, (u1.x * P1.get(2, 2)[0] - P1.get(0, 2)[0]) / wi1);

			A.put(3, 0, (u1.y * P1.get(2, 0)[0] - P1.get(1, 0)[0]) / wi1);
			A.put(3, 1, (u1.y * P1.get(2, 1)[0] - P1.get(1, 1)[0]) / wi1);
			A.put(3, 2, (u1.y * P1.get(2, 2)[0] - P1.get(1, 2)[0]) / wi1);

			Mat B = new Mat(4, 1, CvType.CV_64FC1);
			B.put(0, 0, -(u.x * P.get(2, 3)[0] - P.get(0, 3)[0]) / wi);
			B.put(1, 0, -(u.y * P.get(2, 3)[0] - P.get(1, 3)[0]) / wi);
			B.put(2, 0, -(u1.x * P1.get(2, 3)[0] - P1.get(0, 3)[0]) / wi1);
			B.put(3, 0, -(u1.y * P1.get(2, 3)[0] - P1.get(1, 3)[0]) / wi1);

			Core.solve(A, B, X_, Core.DECOMP_SVD);

			X.put(0, 0, (X_.get(0, 0)[0]));
			X.put(1, 0, (X_.get(1, 0)[0]));
			X.put(2, 0, (X_.get(2, 0)[0]));
			X.put(3, 0, Double.valueOf(1.0));

		}
		return X;
	}

	public static boolean testTriangulation(ArrayList<CloudPoint> pcloud,
			Mat P, ArrayList<Byte> status) {

		ArrayList<Point3> pcloud_pt3d = Tools.cloudPoints2Points3(pcloud);
		if(pcloud_pt3d.isEmpty())return false;
		
		ArrayList<Point3> pcloud_pt3d_projected = new ArrayList<Point3>();

		Mat P4x4 = Mat.eye(4, 4, CvType.CV_64FC1);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				P4x4.put(i, j, P.get(i, j)[0]);
			}
		}
		
		
		
		Mat src = Converters.vector_Point3d_to_Mat(pcloud_pt3d);
		Mat dest = new Mat(src.rows(), src.cols(), src.type());

		Core.perspectiveTransform(src, dest, P4x4);
		
		Converters.Mat_to_vector_Point3d(dest, pcloud_pt3d_projected);
		status.clear();

		for (int i = 0; i < pcloud.size(); i++) {
			status.add(((pcloud_pt3d_projected.get(i).z > 0) ? Byte
					.valueOf((byte) 1) : Byte.valueOf((byte) 0)));
		}
		int count = Collections.frequency(status, Byte.valueOf((byte) 1.0));

		double percentage = ((double) count / (double) pcloud.size());

		if (percentage < 0.55)
			return false;

		// check for coplanarity of points
		/*
		 * if (false) // not { Mat cldm = new Mat(pcloud.size(), 3,
		 * CvType.CV_64FC1); for (int i = 0; i < pcloud.size(); i++) {
		 * cldm.put(i, 0, pcloud.get(i).pt.x); cldm.put(i, 1,
		 * pcloud.get(i).pt.y); cldm.put(i, 2, pcloud.get(i).pt.z);
		 * 
		 * } Mat mean = new Mat(); Mat eigens = new Mat(); Core.PCACompute(cldm,
		 * mean, eigens);
		 * 
		 * int num_inliers = 0; Mat nrm = eigens.row(2).clone(); double
		 * nrmNormed = Core.norm(nrm); for (int i = 0; i < nrm.cols(); i++) {
		 * nrm.put(0, i, nrm.get(0, i)[0] / nrmNormed); }
		 * 
		 * Point3 x0 = new Point3(mean.get(0, 0)[0], mean.get(0, 1)[0],
		 * mean.get(0, 2)[0]); double p_to_plane_thresh =
		 * Math.sqrt(eigens.get(0, 2)[0]);
		 * 
		 * for (int i = 0; i < pcloud.size(); i++) {
		 * 
		 * Mat w = new Mat(1, 3, CvType.CV_64FC1); w.put(0, 0,
		 * pcloud.get(i).pt.x - x0.x); w.put(0, 1, pcloud.get(i).pt.y - x0.y);
		 * w.put(0, 2, pcloud.get(i).pt.z - x0.z);
		 * 
		 * double D = Math.abs(nrm.dot(w)); if (D < p_to_plane_thresh)
		 * num_inliers++; }
		 * 
		 * // cout << num_inliers << "/" << pcloud.size() << " are coplanar" <<
		 * // endl; if ((double) num_inliers / (double) (pcloud.size()) > 0.85)
		 * return false; }
		 */
		return true;
	}

	static Boolean triangulatePointsBetweenViews(int working_view,
			int older_view, ArrayList<CloudPoint> new_triangulated,
			ArrayList<Integer> add_to_cloud) {

		
		Mat P = Box.pMats.get(older_view);
		Mat P1 = Box.pMats.get(working_view);

		ArrayList<KeyPoint> pt_set1 = new ArrayList<KeyPoint>(), pt_set2 = new ArrayList<KeyPoint>();
		ArrayList<DMatch> matches = Box.matchesMap
				.get(new MyPair<Integer, Integer>(older_view, working_view));

		MyMatrix.getAlignedPointsFromMatch(Box.pointsList.get(older_view),
				Box.pointsList.get(working_view), matches, pt_set1, pt_set2);

		// MyMatrix.getAlignedPointsFromMatch(Box.goodPointsList.get(working_view),
		// Box.goodPointsList.get(older_view), matches, pt_set1, pt_set2);

		// adding more triangulated points to general cloud
		double reproj_error = triangulatePoints(
		/*
		 * Box.goodPointsList.get(working_view),
		 * Box.goodPointsList.get(older_view),
		 */pt_set1, pt_set2, Box.intrinsicMat, Box.invIntrinsicMat,
				Box.distortionMat, P, P1, new_triangulated, Box.correspImg1Pt);

		ArrayList<Byte> trig_status = new ArrayList<Byte>();

		if (!testTriangulation(new_triangulated, P, trig_status)
				|| !testTriangulation(new_triangulated, P1, trig_status)) {
			return false;
		}

		// filter out outlier points with high reprojection
		ArrayList<Double> reprj_errors = new ArrayList<Double>();
		for (int i = 0; i < new_triangulated.size(); i++) {
			reprj_errors.add(new_triangulated.get(i).reprojection_error);
		}
		Collections.sort(reprj_errors);
		;
		// get the 80% precentile
		double reprj_err_cutoff = reprj_errors.get(4 * reprj_errors.size() / 5) * 2.4; // threshold
																						// from
																						// Snavely07
																						// 4.2

		ArrayList<CloudPoint> new_triangulated_filtered = new ArrayList<CloudPoint>();
		ArrayList<DMatch> new_matches = new ArrayList<DMatch>();

		for (int i = 0; i < new_triangulated.size(); i++) {
			if (trig_status.get(i) == 0)
				continue; // point was not in front of camera
			if (new_triangulated.get(i).reprojection_error > 16.0) {
				continue; // reject point
			}
			if (new_triangulated.get(i).reprojection_error < 4.0
					|| new_triangulated.get(i).reprojection_error < reprj_err_cutoff) {

				new_triangulated_filtered.add(new_triangulated.get(i));
				new_matches.add(matches.get(i));
			} else {
				continue;
			}
		}

		if (new_triangulated_filtered.size() <= 0)
			return false;

		// use filtered points now
		new_triangulated.clear();

		new_triangulated.addAll(0, new_triangulated_filtered);
		// use filtered matches
		matches = new_matches;

		// update the matches storage
		Box.matchesMap.put(new MyPair<Integer, Integer>(older_view,
				working_view), new_matches); // just to make sure, remove if
												// unneccesary
		Box.matchesMap.put(new MyPair<Integer, Integer>(working_view,
				older_view), MyFeatures.flipMatches(new_matches));

		// now, deterMine which points should be added to the cloud

		add_to_cloud.clear();
		for (int i = add_to_cloud.size(); i < new_triangulated.size(); i++) {
			add_to_cloud.add(1);
		}
		int found_other_views_count = 0;
		int num_views = Box.imagesList.size();

		// scan new triangulated points, if they were already triangulated
		// before - strengthen cloud

		for (int j = 0; j < new_triangulated.size(); j++) {
			for (int k = new_triangulated.get(j).imgpt_for_img.size(); k < Box.imagesList
					.size(); k++) {
				new_triangulated.get(j).imgpt_for_img.add(-1);
			}

			// matches[j] corresponds to new_triangulated[j]
			// matches[j].queryIdx = point in <older_view>
			// matches[j].trainIdx = point in <working_view>
			new_triangulated.get(j).imgpt_for_img.set(older_view,
					matches.get(j).queryIdx); // 2D reference to <older_view>
			new_triangulated.get(j).imgpt_for_img.set(working_view,
					matches.get(j).trainIdx); // 2D reference to <working_view>

			Boolean found_in_other_view = false;
			for (int view_ = 0; view_ < num_views; view_++) {
				if (view_ != older_view && view_ != working_view) {
					// Look for points in <view_> that match to points in
					// <working_view>
					ArrayList<DMatch> submatches = Box.matchesMap
							.get(new MyPair<Integer, Integer>(view_,
									working_view));
					for (int ii = 0; ii < submatches.size(); ii++) {
						if (submatches.get(ii).trainIdx == matches.get(j).trainIdx
								&& !found_in_other_view) {
							// Point was already found in <view_> - strengthen
							// it in the known cloud, if it exists there

							for (int pt3d = 0; pt3d < Box.pcloud.size(); pt3d++) {
								if (Box.pcloud.get(pt3d).imgpt_for_img
										.get(view_) == submatches.get(ii).queryIdx) {
									// pcloud[pt3d] - a point that has 2d
									// reference in <view_>

									{
										Box.pcloud.get(pt3d).imgpt_for_img.set(
												working_view,
												matches.get(j).trainIdx);
										Box.pcloud.get(pt3d).imgpt_for_img.set(
												older_view,
												matches.get(j).queryIdx);
										found_in_other_view = true;
										add_to_cloud.set(j, 0);
									}
								}
							}
						}
					}
				}
			}

			{
				if (found_in_other_view) {
					found_other_views_count++;
				} else {
					add_to_cloud.set(j, 1);
				}
			}
		}

		return true;
	}

}
