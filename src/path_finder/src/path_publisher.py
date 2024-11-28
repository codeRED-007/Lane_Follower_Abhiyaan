#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
import struct
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField



class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('pixel_coord')
        self.subscription = self.create_subscription(Image, '/ipm_image_l', self.image_callback, 10)
        self.subscription = self.create_subscription(Float64,'/lane_width',self.width_callback,10)
        self.publisher = self.create_publisher(PointCloud2,'/goal_points',10)
        self.publisher_vec = self.create_publisher(PointCloud2,'/goal_vec',10)
        self.bridge = CvBridge()
        self.width_received = False

    # def publish_as_vector(self, offset_points):
    #     vector_msg = Float32MultiArray()
        
    #     # Flatten the list of points into a single array
    #     flat_list = [coord for point in offset_points for coord in point]
    #     vector_msg.data = flat_list
        
    #     self.publisher_vec.publish(vector_msg)

    def width_callback(self,msg): 
        self.width = msg.data
        self.width_received = True
    
    def image_callback(self, msg):
        if not self.width_received: return 
        else: self.width_received = False

        image = self.bridge.imgmsg_to_cv2(msg)
        # image = cv2.flip(image,1)
        kernel_size = (7, 7)  # Size of the kernel, (width, height)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)

        # Step 3: Apply dilation
        image = cv2.dilate(image, kernel, iterations=1)
        image = cv2.GaussianBlur(image, (5, 5), 1.5)
        image = cv2.Canny(image, 0, 255, apertureSize=5)

        edge_points = np.column_stack(np.where(image > 0))
        if edge_points.size == 0:
            print("No edge points found.")
            return

        
        sorted_edge_points = edge_points[np.argsort(edge_points[:, 0])]

        # Get the image width
        image_width = image.shape[1]
        filtered_edge_points = sorted_edge_points

        # Take points at regular intervals
        interval_y = 20 # Set the y-interval
        sampled_points = []

        # Traverse from the top of the image to the bottom
        current_y = filtered_edge_points[0][0]
        for y in range(current_y, filtered_edge_points[-1][0], interval_y):
            points_at_y = filtered_edge_points[filtered_edge_points[:, 0] == y]
            if points_at_y.size > 0:
                # Take the average x for all points at this y
                avg_x = int(np.mean(points_at_y[:, 1]))
                sampled_points.append((avg_x, y))

        # Convert to numpy arrays for polynomial fitting
        sampled_points = np.array(sampled_points)

        if len(sampled_points) > 2:  # We need at least 3 points for polynomial fitting
            # Fit a cubic polynomial to the sampled points
            coeffs = np.polyfit(sampled_points[:, 1], sampled_points[:, 0], 3)  # 3rd-degree polynomial
            poly = np.poly1d(coeffs)

            # Generate smooth y-values and corresponding x-values for the original lane
            y_smooth = np.linspace(sampled_points[:, 1].min(), sampled_points[:, 1].max(), 1000)
            x_smooth = poly(y_smooth)

            # Draw the original lane
            for i in range(len(x_smooth) - 1):
                cv2.line(image, (int(x_smooth[i]), int(y_smooth[i])), (int(x_smooth[i + 1]), int(y_smooth[i + 1])), (255, 255, 255), 2)

            # Sample points at regular intervals from the smoothened curve
            sampled_indices = np.linspace(0, len(x_smooth) - 1, 50).astype(int)
            sampled_points_smooth = np.column_stack((x_smooth[sampled_indices], y_smooth[sampled_indices]))



            # Offset the sampled points perpendicularly

            print(sampled_points)
            

            if sampled_points[-1][0] < image_width/2:
                offset = self.width*100/2 # Offset distance
            else:
                offset = -self.width*100/2
            offset_points = []
            print (offset)
            for i in range(0,len(sampled_points_smooth) - 1,8):
                x1, y1 = sampled_points_smooth[i]
                x2, y2 = sampled_points_smooth[i + 1]

                # Calculate the direction vector (dx, dy) of the curve
                dx, dy = x2 - x1, y2 - y1

                # Normalize the direction vector
                length = np.sqrt(dx**2 + dy**2)
                dx /= length
                dy /= length

                # Perpendicular vector (dy, -dx) gives the direction for offset
                perp_x = dy * offset
                perp_y = -dx * offset

                # Offset the point perpendicularly
                offset_x = x1 + perp_x
                offset_y = y1 + perp_y
                # if (i==len(sampled_points_smooth)-1):
                offset_points.append([offset_x, offset_y,0])

                # Draw the offset points as a parallel line
                # if i < len(offset_points) - 1:
                #     cv2.line(image, (int(offset_points[i][0]), int(offset_points[i][1])), (int(offset_points[i + 1][0]), int(offset_points[i + 1][1])), (255,255,255), 2)
        else:
            # Fallback if not enough points for polynomial fitting
            for i in range(len(sampled_points) - 1):
                x1, y1 = sampled_points[i]
                x2, y2 = sampled_points[i + 1]
                cv2.line(image, (x1, y1), (x2, y2), (255, 255, 255), 3)
        try:
            for i in offset_points:
                # Convert the coordinates to integers
                x = int(i[0])
                y = int(i[1])
                
                # Check if the point is within the image boundaries
                if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                    image[y, x] = 255  # For grayscale images (single channel)
                    # For RGB images, use: image[y, x] = (255, 255, 255)  # White color
            
            self.transform_to_pointcloud(offset_points,image)
        except:
            print("Lane not visible")
        

    def transform_to_pointcloud(self,offset_points,image):
        

        for i in offset_points:
            i[0] = image.shape[0] - i[0]
            i[1] = image.shape[1] - i[1]
            # i[1] =  i[1] - image.shape[1]

            i[0] = i[0] - image.shape[0]/2
          
            i[0] = i[0]/100
            i[1] = i[1]/100
           

        header = Header()
        
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link' 

        fields = [
            PointField(name='y', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='x', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = np.array(offset_points, dtype=np.float32).tobytes()

        pointcloud = PointCloud2(
            header=header,
            height=1,
            width=len(offset_points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 4 bytes per float * 3 fields
            row_step=12 * len(offset_points),
            data=cloud_data,
        )

        self.publisher.publish(pointcloud)

        # if len(offset_points) != 0:
        #     self.publish_as_vector(offset_points)

        # if (len(offset_points) != 0):
        #     print (offset_points[0][1],offset_points[0][0],offset_points[0][2])
        #     msg = Float32MultiArray()
        #     msg.data = [float(offset_points[0][1]),float(offset_points[0][0]),float(offset_points[0][2])]
        #     self.publisher_vec.publish(msg)

        # self.transform_to_vector(offset_points,image)
        cv2.imshow("window",image)
        cv2.waitKey(1)
       
    def transform_to_vector(self,offset_points, image):
        # for i in offset_points:
        #     i[0] = image.shape[0] - i[0]
        #     i[1] = image.shape[1] - i[1]
        #     # i[1] =  i[1] - image.shape[1]

        #     i[0] = i[0] - image.shape[0]/2
          
        #     i[0] = i[0]/100
        #     i[1] = i[1]/100
        print ("")

        
        

    

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
