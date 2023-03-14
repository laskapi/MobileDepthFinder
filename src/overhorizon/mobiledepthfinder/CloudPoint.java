package overhorizon.mobiledepthfinder;

import java.util.ArrayList;

import org.opencv.core.Point3;

public class CloudPoint {

	Point3 pt;
	ArrayList<Integer> imgpt_for_img;

	public CloudPoint() {

		imgpt_for_img = new ArrayList<Integer>();
	}

	double reprojection_error;

}
