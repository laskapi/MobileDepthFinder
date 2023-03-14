package overhorizon.mobiledepthfinder;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;

import android.util.Log;

public class MyFeatures {
	private ArrayList<Mat> descriptors;
	private MyProgressDialog progress;

	public enum Methods {

		FAST(1),FAST_PYRAMID(2001),FAST_GRID(1001), ORB(5), ORB_PYRAMID(2005), HARRIS(8);
		private final int value;

		Methods(int value) {

			this.value = value;

		}

		public static String[] getStrings() {
			ArrayList<String> a = new ArrayList<String>();
			for (Methods p : values()) {
				a.add(p.name());
			}
			return a.toArray(new String[a.size()]);
		}

		
	}

	 static Methods method = Methods.FAST;

	// -------------------------------------------------------
	// -------------------------------------------------------

	public MyFeatures(MyProgressDialog progress) {
		descriptors = new ArrayList<Mat>();
		this.progress = progress;

	}

	// -------------------------------------------------------
	// -------------------------------------------------------
	void detectFeatures() {
		progress.setProgress(0);

		Log.i(Box.TAG, "extracting features");
	
		FeatureDetector detector = FeatureDetector.create(method.value);
		
		
		
		DescriptorExtractor extractor = DescriptorExtractor
				.create(DescriptorExtractor.ORB);

		for (int i = 0; i < Box.imagesList.size(); i++) {
			progress.setProgress(((double) i)
					/ ((double) Box.imagesList.size()));
			
			MatOfKeyPoint m = new MatOfKeyPoint();
			detector.detect(Box.imagesList.get(i), m);
			Box.pointsList.add(m.toList());
			Log.i(Box.TAG, "points found:" + m.toArray().length);

			Mat desc = new Mat();
			extractor.compute(Box.imagesList.get(i), m, desc);
			descriptors.add(desc);
			Log.i(Box.TAG, "descriptors in image " + i + "::" + desc.size());
		}

		Log.i(Box.TAG, "features extracted in " + Box.imagesList.size()
				+ "pictures");

	}

	// -------------------------------------------------------
	// -------------------------------------------------------
	void matchFeatures() {
		progress.setProgress(0);
		int progression = 0;
		Mat fund = new Mat();
		// int i=0;
		for (int i = 0; i < Box.imagesList.size() - 1; i++) {
			for (int j = i + 1; j < Box.imagesList.size(); j++) {
				progression++;
				progress.setProgress(((double) progression)
						/ ((double) (Box.imagesList.size()
								* (Box.imagesList.size() + 1) / 2)));
				Log.i(Box.TAG, "matching " + i + " and " + j);

				ArrayList<DMatch> matches = richMatchFeatures(i, j, fund);

				Box.matchesMap.put(new MyPair<Integer, Integer>(i, j), matches);
				ArrayList<DMatch> tempMatches = flipMatches(matches);
				Box.matchesMap.put(new MyPair<Integer, Integer>(j, i),
						tempMatches);
			}
		}

	}

	// -------------------------------------------------------
	// -------------------------------------------------------

	static ArrayList<DMatch> flipMatches(ArrayList<DMatch> matches) {
		ArrayList<DMatch> result = new ArrayList<DMatch>();

		for (DMatch dm : matches) {

			DMatch tempMatch = new DMatch(dm.trainIdx, dm.queryIdx, dm.imgIdx,
					dm.distance);

			result.add(tempMatch);

		}
		return result;
	}

	// -------------------------------------------------------
	// -------------------------------------------------------
	ArrayList<DMatch> richMatchFeatures(int i, int j, Mat fund) {

		ArrayList<DMatch> matches = new ArrayList<DMatch>();
		List<KeyPoint> keypoints_1 = Box.pointsList.get(i);
		List<KeyPoint> keypoints_2 = Box.pointsList.get(j);
		Mat descriptors_1 = descriptors.get(i);
		Mat descriptors_2 = descriptors.get(j);

		ArrayList<DMatch> good_matches_ = new ArrayList<DMatch>();

		if (descriptors_1.empty() || descriptors_2.empty()) {
			return matches;
		}
		// matching descriptor vectors using Brute Force matcher

		DescriptorMatcher matcher = DescriptorMatcher
				.create(DescriptorMatcher.BRUTEFORCE_HAMMING);

		// ArrayList<DMatch> matches_;
		ArrayList<Float> dists = new ArrayList<Float>();

		/*
		 * if (matches.size() == 0) { ArrayList<MatOfDMatch> knnMatches = new
		 * ArrayList<MatOfDMatch>(); matcher.knnMatch(descriptors_1,
		 * descriptors_2, knnMatches, 1); matches.clear(); for (int ii = 0; ii <
		 * knnMatches.size(); ii++) { if (!knnMatches.get(ii).empty()) { DMatch
		 * tempMatch = knnMatches.get(ii).toArray()[0]; float dist =
		 * tempMatch.distance;
		 * 
		 * if (Math.abs(dist) > 10000) dist = 1; tempMatch.distance = dist;
		 * 
		 * 
		 * matches.add(tempMatch);
		 * 
		 * dists.add(dist); } } }
		 */
		if (matches.size() == 0) {
			ArrayList<MatOfDMatch> knnMatches12 = new ArrayList<MatOfDMatch>();
			ArrayList<MatOfDMatch> knnMatches21 = new ArrayList<MatOfDMatch>();
			matcher.knnMatch(descriptors_1, descriptors_2, knnMatches12, 1);
			matcher.knnMatch(descriptors_2, descriptors_1, knnMatches21, 1);

			matches.clear();

			for (int ii = 0; ii < knnMatches12.size(); ii++) {
				if (!knnMatches12.get(ii).empty()) {

					DMatch forward = knnMatches12.get(ii).toArray()[0];

					DMatch backward = knnMatches21.get(forward.trainIdx)
							.toArray()[0];
					if (backward.trainIdx == forward.queryIdx) {
						float dist = forward.distance;
						if (Math.abs(dist) > 10000) {
							dist = 1;
						}
						forward.distance = dist;

						matches.add(forward);
						dists.add(dist);
					}
				}
			}
		}

		// ---------------------------------------

		// ------------albo
		/*
		 * BFMatcher matcher(NORM_L2,true); matcher.match( descriptors_1,
		 * descriptors_2, *matches );
		 */// --------------------

		double min_dist = Tools.findMin(dists);

		ArrayList<KeyPoint> imgpts1_good = new ArrayList<KeyPoint>(), imgpts2_good = new ArrayList<KeyPoint>();

		if (min_dist < 10.0) {
			min_dist = 10.0;
		}

		// Eliminate any re-matching of training points (multiple queries to one
		// training)
		double cutoff = 4.0 * min_dist;
		HashSet<Integer> existing_trainIdx = new HashSet<Integer>();
		for (int iii = 0; iii < matches.size(); iii++) {
			// "normalize" matching: somtimes imgIdx is the one holding the
			// trainIdx
			if (matches.get(iii).trainIdx <= 0) {
				matches.get(iii).trainIdx = matches.get(iii).imgIdx;
			}

			int tidx = matches.get(iii).trainIdx;
			if (matches.get(iii).distance > 0.0
					&& matches.get(iii).distance < cutoff) {
				if ((!existing_trainIdx.contains(tidx)) && tidx >= 0
						&& tidx < (int) (keypoints_2.size())) {
					good_matches_.add(matches.get(iii));
					imgpts1_good.add(keypoints_1.get(matches.get(iii).queryIdx));
					imgpts2_good.add(keypoints_2.get(tidx));
					existing_trainIdx.add(tidx);
				}
			}
		}
		ArrayList<KeyPoint> imgpts1_very_good = new ArrayList<KeyPoint>(), imgpts2_very_good = new ArrayList<KeyPoint>();
		MultiValue<ArrayList<DMatch>, Mat, Mat, Mat> result = MyMatrix
				.getFundamentalMat(keypoints_1, keypoints_2, imgpts1_very_good,
						imgpts2_very_good, good_matches_);

		return result.getFirst();
		// return good_matches_;
	}

	// -------------------------------------------------------
	// -------------------------------------------------------
	public static Mat getFeature(Mat src) {
		FeatureDetector detector = FeatureDetector.create(method.value);

		Mat dst = Mat.zeros(src.rows(), src.cols(), CvType.CV_8UC1);
		MatOfKeyPoint m = new MatOfKeyPoint();
		detector.detect(src, m);
		List<KeyPoint> list = m.toList();
		for (KeyPoint k : list) {
			dst.put((int) k.pt.y, (int) k.pt.x, 255);
		}

		return dst;
	}

	// -------------------------------------------------------
	// -------------------------------------------------------
	public static void setMethod(int which) {
		for (Methods m : Methods.values()) {
			if (m.ordinal() == which) {
				method = m;
				return;
			}
		}

	}

}
