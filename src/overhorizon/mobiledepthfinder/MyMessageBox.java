package overhorizon.mobiledepthfinder;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;

public class MyMessageBox {
//	private Activity parent;

	public MyMessageBox(Activity parent, String title, final MyFun onOk,
			final MyFun onCancel) {
//		this.parent = parent;

		AlertDialog.Builder builder = new AlertDialog.Builder(parent);
		builder.setTitle(title).setIcon(android.R.drawable.ic_dialog_info)
				.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int which) {
						if (onOk != null) {
							onOk.use(which);
						}
						dialog.dismiss();
					}
				});

		if (onCancel != null) {
			builder.setNegativeButton("Cancel",
					new DialogInterface.OnClickListener() {
						public void onClick(DialogInterface dialog, int which) {
							onCancel.use(which);

							dialog.dismiss();
						}
					});
		}

		AlertDialog alert = builder.create();
		alert.show();

	}

}