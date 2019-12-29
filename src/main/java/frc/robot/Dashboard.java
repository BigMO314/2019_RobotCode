package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;;

public class Dashboard {
	public static void clear() { NetworkTableInstance.getDefault().deleteAllEntries(); }
	
	private static abstract class Generic {
		protected String key;
		public void delete() { SmartDashboard.delete(key); }
	}
	
	public static class NumberEntry extends Generic {
		private double defaultValue;
		public NumberEntry(String key) { this(key, 0.0); }
		public NumberEntry(String key, double defaultValue) {
			this.key = key;
			this.defaultValue = defaultValue;
			this.set(defaultValue);
		}
		
		public void set(double value) { SmartDashboard.putNumber(key, value); }
		public double get() { return SmartDashboard.getNumber(key, defaultValue); }
	}
	
	public static class StringEntry extends Generic {
		private String defaultValue;
		public StringEntry(String key) { this(key, ""); }
		public StringEntry(String key, String defaultValue) {
			this.key = key;
			this.defaultValue = defaultValue;
			this.set(defaultValue);
		}
		
		public void set(String value) { SmartDashboard.putString(key, value); }
		public String get() { return SmartDashboard.getString(key, defaultValue); }
	}
	
	public static class BooleanEntry extends Generic {
		private boolean defaultValue;
		public BooleanEntry(String key) { this(key, false); }
		public BooleanEntry(String key, boolean defaultValue) {
			this.key = key;
			this.defaultValue = defaultValue;
			this.set(defaultValue);
		}
		
		public void set(boolean value) { SmartDashboard.putBoolean(key, value); }
		public boolean get() { return SmartDashboard.getBoolean(key, defaultValue); }
	}
	
	public static class CheckBox extends BooleanEntry {
		public CheckBox(String key) { this(key, false); }
		public CheckBox(String key, boolean defaultValue) { super(key, defaultValue); }

		public void check() { set(true); }
		public void uncheck() { set(false); }
	}

	public static class Indicator extends BooleanEntry {
		public Indicator(String key) { this(key, false); }
		public Indicator(String key, boolean defaultValue) { super(key, defaultValue); }

		public void turnOn() { set(true); }
		public void turnOff() { set(false); }
	}
}