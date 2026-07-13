import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

try:
    from tf.transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(roll, pitch, yaw):
        return (0.0, 0.0, 0.0, 1.0)


def gps_to_cartesian(origin_lat, origin_lon, dest_lat, dest_lon):
    """
    Convert GPS coordinates to Cartesian coordinates (x, y).
    
    Args:
        origin_lat: Initial latitude (origin of reference frame)
        origin_lon: Initial longitude (origin of reference frame)
        dest_lat: Destination latitude
        dest_lon: Destination longitude
    
    Returns:
        Tuple of (x, y) coordinates in meters
    """
    # Earth's radius in meters
    R = 6371000
    
    # Convert degrees to radians
    lat1_rad = math.radians(origin_lat)
    lon1_rad = math.radians(origin_lon)
    lat2_rad = math.radians(dest_lat)
    lon2_rad = math.radians(dest_lon)
    
    # Calculate differences
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # Calculate Cartesian coordinates
    # x is the east-west displacement (longitude)
    # y is the north-south displacement (latitude)
    x = R * dlon * math.cos(lat1_rad)
    y = R * dlat
    
    return x, y


def main():
    print("GPS to Cartesian Converter")
    print("=" * 40)
    
    # Input origin coordinates
    print("\nEnter initial GPS coordinates (origin):")
    try:
        origin_lat = float(input("Initial Latitude: "))
        origin_lon = float(input("Initial Longitude: "))
    except ValueError:
        print("Error: Please enter valid numeric coordinates.")
        return
    
    # Input destination coordinates
    print("\nEnter destination GPS coordinates:")
    try:
        dest_lat = float(input("Destination Latitude: "))
        dest_lon = float(input("Destination Longitude: "))
    except ValueError:
        print("Error: Please enter valid numeric coordinates.")
        return
    
    # Convert GPS to Cartesian
    x, y = gps_to_cartesian(origin_lat, origin_lon, dest_lat, dest_lon)
    
    # Output results
    print("\n" + "=" * 40)
    print("Results:")
    print(f"Origin: ({origin_lat}, {origin_lon})")
    print(f"Destination: ({dest_lat}, {dest_lon})")
    print(f"Cartesian Coordinates (origin at {origin_lat}, {origin_lon}):")
    print(f"X (East-West): {x:.2f} meters")
    print(f"Y (North-South): {y:.2f} meters")
    print(f"Distance: {math.sqrt(x**2 + y**2):.2f} meters")


def create_goal_pose(x, y, yaw=0.0, frame_id="map"):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = 0.0

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    return pose_stamped


def main():
    rospy.init_node("gps2cart_goal_publisher", anonymous=True)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    print("GPS to Cartesian Goal Publisher")
    print("=" * 40)

    print("\nEnter initial GPS coordinates (origin):")
    try:
        origin_lat = float(input("Initial Latitude: "))
        origin_lon = float(input("Initial Longitude: "))
    except ValueError:
        print("Error: Please enter valid numeric coordinates.")
        return

    print("Enter destination GPS coordinates:")
    try:
        dest_lat = float(input("Destination Latitude: "))
        dest_lon = float(input("Destination Longitude: "))
    except ValueError:
        print("Error: Please enter valid numeric coordinates.")
        return

    x, y = gps_to_cartesian(origin_lat, origin_lon, dest_lat, dest_lon)
    distance = math.hypot(x, y)
    yaw = math.atan2(y, x)

    goal = create_goal_pose(x, y, yaw=yaw, frame_id="map")

    rospy.sleep(1.0)
    pub.publish(goal)
    rospy.loginfo("Published goal: x=%.2f y=%.2f distance=%.2f", x, y, distance)

    print("\nGoal published to /move_base_simple/goal")
    print(f"Cartesian target: x={x:.2f} m, y={y:.2f} m")
    print(f"Distance: {distance:.2f} m")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
