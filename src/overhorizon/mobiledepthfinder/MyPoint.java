package overhorizon.mobiledepthfinder;

import org.opencv.core.Point;

public class MyPoint extends Point implements Comparable<MyPoint>{

	public MyPoint(double x,double y) {
	super(x,y);
	}
	
	public MyPoint(Point p) {
		super(p.x,p.y);
	}
		
	@Override
	public int compareTo(MyPoint another) {
		double result= this.getValue()-another.getValue();
		if (result>0) return 1;
		else if (result<0) return -1;
		else return 0;
	}
	 double getValue(){
		
		return Math.sqrt(x*x+y*y);
		
	}
	

}
