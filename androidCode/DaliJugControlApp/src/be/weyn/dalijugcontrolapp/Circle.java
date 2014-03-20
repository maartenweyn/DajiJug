package be.weyn.dalijugcontrolapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Paint.Align;
import android.view.View;

public class Circle extends View {
	
	private float x;
    private float y;
    private final float r;
    private final Paint mPaint = new Paint (Paint.ANTI_ALIAS_FLAG);
    private final Paint mPaintCross = new Paint (Paint.ANTI_ALIAS_FLAG);
    private boolean cross;
    private String text;
	
	public Circle(Context context, float x, float y, float r, int color, boolean cross, String text) {
		super(context);
		mPaint.setColor(color);
		mPaintCross.setColor(0xA0FFFFFF);
		mPaintCross.setTextSize(20);
		mPaintCross.setTextAlign(Align.CENTER);
        this.x = x;
        this.y = y;
        this.r = r;
        this.cross = cross;
        this.text = text;
	}
	
	public Circle(Context context) {
		super(context);
		mPaint.setColor(0xFF000000);
		mPaintCross.setColor(0xA0FFFFFF);
		mPaintCross.setTextSize(20);
		mPaintCross.setTextAlign(Align.CENTER);
        this.x = 0;
        this.y = 0;
        this.r = 10;
        this.text = "";
	}
	
	public void Move(float d, float e)
	{
		this.x = d;
		this.y = e;
		
		this.invalidate();
	}

	@Override 
	protected void onDraw(Canvas canvas) {
		super.onDraw(canvas); 
		
		canvas.drawCircle(x, y, r, mPaint);
		canvas.drawText(text, x, y, mPaintCross);
		if(cross)
		{
			canvas.drawLine(x-r, y, x+r, y, mPaintCross);
			canvas.drawLine(x, y-r, x, y+r, mPaintCross);
		}
    }
}