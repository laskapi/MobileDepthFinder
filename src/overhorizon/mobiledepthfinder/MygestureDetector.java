package overhorizon.mobiledepthfinder;

import android.content.Context;
import android.os.Handler;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;

class MyGestureDetector extends GestureDetector.SimpleOnGestureListener {
	Handler handler;
	private Context mContext;

	public MyGestureDetector(Handler handler, Context c) {
		super();
		this.handler = handler;
		this.mContext = c;
	}

	@Override
	public boolean onDoubleTapEvent(final MotionEvent e) {
		if (e.getActionMasked() == MotionEvent.ACTION_UP) {
			new Thread(new Runnable() {

				@Override
				public void run() {

				View mView = ((DepthFinder) mContext)
							.findViewById(R.id.cameraView);
					int[] loc = { 0, 0 };
					mView.getLocationOnScreen(loc);

					if (e.getX() - loc[0] > Box.size.width / 2) {
						MyProjection.xPos += 1;

					} else {
						MyProjection.xPos -= 1;
					}
					if (e.getY() - loc[1] > Box.size.height / 2) {
						MyProjection.yPos += 1;

					} else {
						MyProjection.yPos -= 1;
					}

					MyProjection.getResults(handler);

				}
			}).start();
		}

		return true;
	}

	@Override
	public boolean onScroll(final MotionEvent e1, final MotionEvent e2,
			final float distanceX, final float distanceY) {
	
				if (Math.abs(e2.getX() - e1.getX()) < Math.abs(e2.getY()
						- e1.getY())) {

					MyProjection.xAngle += distanceX;
					if (MyProjection.xAngle > 359) {
						MyProjection.xAngle = 359.;
					} else if (MyProjection.xAngle < -359) {
						MyProjection.xAngle = -359.;
					}
				} else {

					MyProjection.yAngle += distanceY;
					if (MyProjection.yAngle > 359) {
						MyProjection.yAngle = 359.;
					} else if (MyProjection.yAngle < -359) {
						MyProjection.yAngle = -359.;
					}
				}
				MyProjection.getResults(handler);

		 try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
				e.printStackTrace();
		}
		 return true;
	}

	@Override
	public boolean onDown(MotionEvent e) {

		return true;
	}

}
