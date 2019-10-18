import pandas as pd
import rospy
import rosbag
from mavros_msgs.msg import Altitude
from std_msgs.msg import Int32, Float64, String

def csv2rosbag():
	log = pd.read_csv('flight_record.csv')
	try:
		with rosbag.Bag('test.bag', 'w') as bag:
		    for row in range(log.shape[0]):
		        timestamp = rospy.Time.from_sec(log['Time(seconds)'][row])
		        altitude_msg = Altitude()
		        altitude_msg.header.stamp = timestamp
		        altitude_msg.local = (log['Altitude(meters)'][row])

		        # Populate the data elements for IMU
		        # e.g. imu_msg.angular_velocity.x = df['a_v_x'][row]

		        bag.write("/mavros/altitude", altitude_msg, timestamp)
		        print(bag)
	finally:
		bag.close()

if __name__ == '__main__':
	csv2rosbag()