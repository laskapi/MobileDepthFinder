package overhorizon.mobiledepthfinder;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.features2d.DMatch;

import android.util.Log;

public class MyReconFill {
	static HashSet<Integer> done_views, good_views;
	static {
		done_views = new HashSet<Integer>();
		good_views = new HashSet<Integer>();
	}

	static void fillRecon(MyProgressDialog progress) {
		progress.setProgress(0);
		Mat P1 = Box.pMats.get(Box.m_second_view);

		Mat t = new Mat(1, 3, CvType.CV_64FC1);
		t.put(0, 0, P1.get(0, 3)[0]);
		t.put(0, 1, P1.get(1, 3)[0]);
		t.put(0, 2, P1.get(2, 3)[0]);

		Mat R = new Mat(3, 3, CvType.CV_64FC1);

		R.put(0, 0, P1.get(0, 0)[0]);
		R.put(0, 1, P1.get(0, 1)[0]);
		R.put(0, 2, P1.get(0, 2)[0]);
		R.put(1, 0, P1.get(1, 0)[0]);
		R.put(1, 1, P1.get(1, 1)[0]);
		R.put(1, 2, P1.get(1, 2)[0]);
		R.put(2, 0, P1.get(2, 0)[0]);
		R.put(2, 1, P1.get(2, 1)[0]);
		R.put(2, 2, P1.get(2, 2)[0]);

		Mat rvec = new Mat(1, 3, CvType.CV_64FC1);
		Calib3d.Rodrigues(R, rvec);

		done_views.add(Box.m_first_view);
		done_views.add(Box.m_second_view);
		good_views.add(Box.m_first_view);
		good_views.add(Box.m_second_view);

		// loop images to incrementally recover more cameras
		 for (int ii=0; ii < Box.imagesList.size(); ii++){
	
	//	while (done_views.size() != Box.imagesList.size()) {
			// find image with highest 2d-3d correspondance [Snavely07 4.2]
			Log.i(Box.TAG,"we 're done with "+done_views.size()+" of "+Box.imagesList.size() +"images");
			int max_2d3d_view = -1, max_2d3d_count = 0;
			
			ArrayList<Point3> max_3d=new ArrayList<Point3>();
			ArrayList<Point> max_2d=new ArrayList<Point>();
			for (int _i = 0; _i < Box.imagesList.size(); _i++) {
				if (done_views.contains(_i))
					continue; // already done with this view

				ArrayList<Point3> tmp_3d=new ArrayList<Point3>();
				ArrayList<Point> tmp_2d=new ArrayList<Point>();

				

				find2D3DCorrespondences(_i,tmp_3d, tmp_2d);
				if (tmp_3d.size() > max_2d3d_count) {
					max_2d3d_count = tmp_3d.size();
					max_2d3d_view = _i;
					max_3d = tmp_3d;
					max_2d = tmp_2d;
				}
			}
			int i = max_2d3d_view; // highest 2d3d matching view
	
			done_views.add(i); // don't repeat it for now
			progress.setProgress(((double)done_views.size())/((double)Box.imagesList.size()));
			
			Boolean pose_estimated = findPoseEstimation(i, rvec, t, R, max_3d,
					max_2d);
			if (!pose_estimated)
				continue;

			// store estimated pose

			Mat tmpMat = new Mat(3, 4, CvType.CV_64FC1);
			tmpMat.put(0, 0, R.get(0, 0)[0]);
			tmpMat.put(0, 1, R.get(0, 1)[0]);
			tmpMat.put(0, 2, R.get(0, 2)[0]);
			tmpMat.put(0, 3, t.get(0, 0)[0]);

			tmpMat.put(1, 0, R.get(1, 0)[0]);
			tmpMat.put(1, 1, R.get(1, 1)[0]);
			tmpMat.put(1, 2, R.get(1, 2)[0]);
			tmpMat.put(1, 3, t.get(0, 1)[0]);

			tmpMat.put(2, 0, R.get(2, 0)[0]);
			tmpMat.put(2, 1, R.get(2, 1)[0]);
			tmpMat.put(2, 2, R.get(2, 2)[0]);
			tmpMat.put(2, 3, t.get(0, 2)[0]);

			Box.pMats.put(i, tmpMat);

			// start triangulating with previous GOOD views
			for (Integer done_view : good_views) {
				int view = done_view;
				if (view == i)
					continue; // skip current...

				// cout << " -> " << view << endl;
				Log.i(Box.TAG,"add points from image "+i);
				ArrayList<CloudPoint> new_triangulated = new ArrayList<CloudPoint>();
				ArrayList<Integer> add_to_cloud = new ArrayList<Integer>();
				Boolean good_triangulation = MyTriangulation
						.triangulatePointsBetweenViews(i, view,
								new_triangulated, add_to_cloud);
				if (!good_triangulation)
					continue;

				// std::cout << "before triangulation: " <<
				// Datas::pcloud.size();
				for (int j = 0; j < add_to_cloud.size(); j++) {
					if (add_to_cloud.get(j) == 1)
						Box.pcloud.add(new_triangulated.get(j));
				}
				// std::cout << " after " << Datas::pcloud.size() << std::endl;
				// break;
			}
			good_views.add(i);

			// BundleAdjuster::AdjustCurrentBundle();
		}
	}

	static void find2D3DCorrespondences(int working_view, List<Point3> ppcloud,
			ArrayList<Point> imgPoints)

	{
		ppcloud.clear();
		imgPoints.clear();

		ArrayList<Integer> pcloud_status = new ArrayList<Integer>();
		for (int i = 0; i < Box.pcloud.size(); i++) {
			pcloud_status.add(new Integer(0));
		}

		for (Integer done_view : good_views) {
			int old_view = done_view;
			// check for matches_from_old_to_working between i'th frame and
			// <old_view>'th frame (and thus the current cloud)
			ArrayList<DMatch> matches_from_old_to_working = Box.matchesMap
					.get(new MyPair<Integer, Integer>(old_view, working_view));

			for (int match_from_old_view = 0; match_from_old_view < matches_from_old_to_working
					.size(); match_from_old_view++) {
				// the index of the matching point in <old_view>
				int idx_in_old_view = matches_from_old_to_working
						.get(match_from_old_view).queryIdx;

				// scan the existing cloud (pcloud) to see if this point from
				// <old_view> exists
				for (int pcldp = 0; pcldp < Box.pcloud.size(); pcldp++) {
					// see if corresponding point was found in this point
					if (idx_in_old_view == Box.pcloud.get(pcldp).imgpt_for_img
							.get(old_view) && pcloud_status.get(pcldp) == 0) // prevent
																				// duplicates
					{
						// 3d point in cloud
						ppcloud.add(Box.pcloud.get(pcldp).pt);
						// 2d point in image i
						imgPoints.add(Box.pointsList.get(working_view).get(
								matches_from_old_to_working
										.get(match_from_old_view).trainIdx).pt);

						pcloud_status.set(pcldp, 1);
						break;
					}
				}
			}
		}
		// cout << "found " << ppcloud.size() <<
		// " 3d-2d point correspondences"<<endl;
	}

	static Boolean findPoseEstimation(
	int working_view,
	Mat rvec,
	Mat t,
	Mat R,
	ArrayList<Point3> max_3d,
	ArrayList<Point> imgPoints
	) 
{
	if(max_3d.size() <= 7 || imgPoints.size() <= 7 || max_3d.size() != imgPoints.size()) { 
		//something went wrong aligning 3D to 2D points..
	//	cerr << "couldn't find [enough] corresponding cloud points... (only " << ppcloud.size() << ")" <<endl;
		return false;
	}

	ArrayList<Integer> inliers=new ArrayList<Integer>();
	//if(!use_gpu) {
	//	//use CPU
	//	double minVal,maxVal; cv::minMaxIdx(imgPoints,&minVal,&maxVal);
	//	CV_PROFILE("solvePnPRansac",cv::solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, 1000, 0.006 * maxVal, 0.25 * (double)(imgPoints.size()), inliers, CV_EPNP);)
	//	//CV_PROFILE("solvePnP",cv::solvePnP(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, CV_EPNP);)
	//}	else
/*	{
#ifdef HAVE_OPENCV_GPU
		//use GPU ransac
		//make sure datatstructures are cv::gpu compatible
		cv::Mat ppcloud_m(ppcloud); ppcloud_m = ppcloud_m.t();
		cv::Mat imgPoints_m(imgPoints); imgPoints_m = imgPoints_m.t();
		cv::Mat rvec_,t_;

		cv::gpu::solvePnPRansac(ppcloud_m,imgPoints_m,K_32f,distcoeff_32f,rvec_,t_,false);

		rvec_.convertTo(rvec,CV_64FC1);
		t_.convertTo(t,CV_64FC1);
#endif
	}*/

	MatOfPoint2f projected3D=new MatOfPoint2f();
	MatOfPoint3f max_3dList= new MatOfPoint3f();
	max_3dList.fromList(max_3d);
	Calib3d.projectPoints(max_3dList, rvec, t, Box.intrinsicMat, new MatOfDouble(Box.distortionMat), projected3D);

	
	if(inliers.size()==0) { //get inliers
	List<Point> projected3DList=projected3D.toList();
		for(int i=0;i<projected3DList.size();i++) {
			MyPoint p=new MyPoint(projected3DList.get(i).clone());
			p.x-=-imgPoints.get(i).x;
			p.y-=-imgPoints.get(i).y;
			if(p.getValue() < 10.0)
				inliers.add(i);
		}
	}

/*#if 0
	//display reprojected points and matches
	cv::Mat reprojected; imgs_orig[working_view].copyTo(reprojected);
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::line(reprojected,imgPoints[ppt],projected3D[ppt],cv::Scalar(0,0,255),1);
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::line(reprojected,imgPoints[inliers[ppt]],projected3D[inliers[ppt]],cv::Scalar(0,0,255),1);
	}
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::circle(reprojected, imgPoints[ppt], 2, cv::Scalar(255,0,0), CV_FILLED);
		cv::circle(reprojected, projected3D[ppt], 2, cv::Scalar(0,255,0), CV_FILLED);			
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::circle(reprojected, imgPoints[inliers[ppt]], 2, cv::Scalar(255,255,0), CV_FILLED);
	}
	stringstream ss; ss << "inliers " << inliers.size() << " / " << projected3D.size();
	putText(reprojected, ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255), 2);

	cv::imshow("__tmp", reprojected);
	cv::waitKey(0);
	cv::destroyWindow("__tmp");
#endif*/
	//cv::Rodrigues(rvec, R);
	//visualizerShowCamera(R,t,0,255,0,0.1);

	if(inliers.size() < (double)(imgPoints.size())/5.0) {
	//	cerr << "not enough inliers to consider a good pose ("<<inliers.size()<<"/"<<imgPoints.size()<<")"<< endl;
		return false;
	}

	if(Core.norm(t) > 200.0) {
		// this is bad...
	//	cerr << "estimated camera movement is too big, skip this camera\r\n";
		return false;
	}

	Calib3d.Rodrigues(rvec, R);
	if(!MyMatrix.CheckCoherentRotation(R)) {
	//	cerr << "rotation is incoherent. we should try a different base view..." << endl;
		return false;
	}

//	std::cout << "found t = " << t << "\nR = \n"<<R<<std::endl;
	return true;
}

}