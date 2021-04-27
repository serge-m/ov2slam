#!/usr/bin/env python
"""
    images_to_rostopic.py 
    Copyright (C) Sergey Matyunin, 2021

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
import cv2
import rospy
from sensor_msgs.msg import Image


def image_to_message(img, stamp):
    msg = Image()
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.step = img.strides[0]
    msg.encoding = 'bgr8'
    msg.header.frame_id = 'image_rect'
    msg.header.stamp = stamp
    msg.data = img.flatten().tolist()
    return msg

   

def main():
    import argparse
    import glob

    parser = argparse.ArgumentParser(description='Publish stereo images sequence to ROS')
    parser.add_argument('-l', 
                        help='Path to left images as glob pattern. Images will be sorted lexicographically', 
                        metavar='PATTERN_LEFT', dest='pattern_left',
                        default='left/*.jpg')
    parser.add_argument('-r', 
                        help='Path to right images as glob pattern. Images will be sorted lexicographically', 
                        metavar='PATTERN_RIGHT', dest='pattern_right',
                        default='right/*.jpg')
    parser.add_argument('--left-topic', 
                        help='The topic name for the left camera', 
                        dest='topic_left',
                        metavar='TOPIC_LEFT', default='/left/image_color')
    parser.add_argument('--right-topic', help='The topic name for the right camera', dest='topic_right',
                        metavar='TOPIC_RIGHT', default='/right/image_color')
    parser.add_argument('--rate', help='Publishing rate', dest='rate', default=1.0, type=float)
    parser.add_argument('--verbose', help='Enables debug output', dest='verbose', action='store_true')
    args = parser.parse_args()

    if args.verbose:
        print("Loading image lists from {}, {}".format(args.pattern_left, args.pattern_right))
    
    left_files = sorted(glob.glob(args.pattern_left))
    right_files = sorted(glob.glob(args.pattern_right))
    cnt_left = len(left_files)
    cnt_right = len(right_files)

    if args.verbose:
        print("Found images left {}, right {}".format(cnt_left, cnt_right))
    
    
    if cnt_left != cnt_right:
        raise ValueError("Number of left and right frames doesn't match. Left: {}, right: {}".format(cnt_left, cnt_right))

    if cnt_left == 0:
        raise ValueError("No images found in {}".format(args.pattern_left))

    rospy.init_node('stereo_publisher')    
    left_img_pub = rospy.Publisher(args.topic_right, Image, queue_size=1)
    right_img_pub = rospy.Publisher(args.topic_right, Image, queue_size=1)
    

    r = rospy.Rate(args.rate)

    for left_file, right_file in zip(left_files, right_files):
        if rospy.is_shutdown():
            break
        left_image = cv2.imread(left_file)
        right_image = cv2.imread(right_file)
        
        stamp = rospy.Time.now()

        left_img_msg = image_to_message(left_image, stamp)
        right_img_msg = image_to_message(right_image, stamp)

        left_img_pub.publish(left_img_msg)
        right_img_pub.publish(right_img_msg)
        
        r.sleep()


if __name__ == '__main__':
    main()
    