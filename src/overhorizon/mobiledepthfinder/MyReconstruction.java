package overhorizon.mobiledepthfinder;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.os.Handler;

public abstract class MyReconstruction  {
	
	
	protected float focal;
	public MyProgressDialog progress;
	

	public MyReconstruction(float focal,MyProgressDialog progress) {
	
		this.focal = focal;
		this.progress=progress;
	}


	abstract void start();

		
}
