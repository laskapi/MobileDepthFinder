package overhorizon.mobiledepthfinder;

import android.app.ProgressDialog;
import android.content.Context;
import android.os.Handler;

public class MyProgressDialog extends ProgressDialog {
	 Handler handler;
	 DepthFinder parent;

	MyProgressDialog(Context context, Handler handler) {
		super(context);
		parent = (DepthFinder) context;
		this.handler = handler;

		setProgressStyle(ProgressDialog.STYLE_HORIZONTAL);
		super.setProgress(0);
		setMax(100);
		MyProgressDialog.super.setMessage("progress");

	}

	public void setProgress(final double value) {

		handler.post(new Runnable() {

			@Override
			public void run() {

				MyProgressDialog.super.setProgress((int)(value*100));
			}
		});

	}

	public void setMessage(final String message) {
		MyProgressDialog.super.setProgress(0);

		handler.post(new Runnable() {

			@Override
			public void run() {
				
				MyProgressDialog.super.setMessage(message);
			}
		});


	}


}